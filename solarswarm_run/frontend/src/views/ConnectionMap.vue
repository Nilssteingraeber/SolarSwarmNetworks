<script setup lang="ts">
import { ref, onMounted, onUnmounted, shallowRef } from 'vue'
import * as d3 from 'd3'
import { useDroneHistoryStore } from '../DronesData/DroneHistoryStore'
import type { Robot } from '../models/Robot'
import { useTimeStore } from '../stores/TimeStore'
import { OhVueIcon } from "oh-vue-icons";

interface ProcessedRobot extends Robot {
    time_since_last_heard: number
    is_stale: boolean
}

const props = defineProps<{ width?: number; height?: number }>()
const historyStore = useDroneHistoryStore()
const timeStore = useTimeStore()

// --- DOM & STATE ---
const containerRef = ref<HTMLDivElement | null>(null)
const svgRef = ref<SVGSVGElement | null>(null)
const containerWidth = ref(props.width || 400)
const containerHeight = ref(props.height || 400)

// Data State
const robots = shallowRef<ProcessedRobot[]>([])

// Camera State
const isAutoMode = ref(true)
const currentBounds = { minLon: 0, maxLon: 0, minLat: 0, maxLat: 0 }
const targetBounds = { minLon: 0, maxLon: 0, minLat: 0, maxLat: 0 }
let isInitialized = false

// Loop Handles
let dataInterval: ReturnType<typeof setInterval> | null = null
let animationFrameId: number | null = null
let resizeObserver: ResizeObserver | null = null

// D3 Objects
let svgSelect: d3.Selection<SVGSVGElement, unknown, null, undefined>
let gGrid: d3.Selection<SVGGElement, unknown, null, undefined>
let gContent: d3.Selection<SVGGElement, unknown, null, undefined>
let zoomBehavior: d3.ZoomBehavior<SVGSVGElement, unknown>

// Colors
const linkColorScale = d3.scaleLinear<string>().domain([0, 0.5, 1]).range(['#FF4444', '#FFFF00', '#00FF00'])
const getNodeColor = (t: number) => t < 3 ? '#00FF00' : t < 5 ? '#FFFF00' : t < 7 ? '#FF0000' : '#888'

// ----------------------------------------------------
// 1. DATA UPDATE LOOP
// ----------------------------------------------------
const updateData = async () => {
    const currentTime = timeStore.currentTime
    const dronesMap = await historyStore.getDronesInWindow(currentTime, 3000)

    const snapshot: ProcessedRobot[] = []
    let minLon = Infinity, maxLon = -Infinity
    let minLat = Infinity, maxLat = -Infinity
    let hasValidPoints = false

    dronesMap.forEach((robot) => {
        if (!robot.point) return;
        const timeSince = Math.abs((currentTime - (robot.last_heard * 1000)) / 1000)

        snapshot.push({
            ...robot,
            time_since_last_heard: timeSince,
            is_stale: timeSince > 5
        })

        // FIX: Calculate bounds for ALL drones, even stale ones. 
        // We only exclude 0,0 points which are likely invalid GPS inits.
        if (Math.abs(robot.point.lon) > 0.001 && Math.abs(robot.point.lat) > 0.001) {
            if (robot.point.lon < minLon) minLon = robot.point.lon
            if (robot.point.lon > maxLon) maxLon = robot.point.lon
            if (robot.point.lat < minLat) minLat = robot.point.lat
            if (robot.point.lat > maxLat) maxLat = robot.point.lat
            hasValidPoints = true
        }
    })

    robots.value = snapshot

    // If in Auto Mode, update where we WANT the camera to be
    if (hasValidPoints && isAutoMode.value) {
        // Prevent singular bounds (if only 1 drone)
        if (maxLon === minLon) { minLon -= 0.0005; maxLon += 0.0005 }
        if (maxLat === minLat) { minLat -= 0.0005; maxLat += 0.0005 }

        // Add 20% padding
        const lonPad = (maxLon - minLon) * 0.2
        const latPad = (maxLat - minLat) * 0.2

        targetBounds.minLon = minLon - lonPad
        targetBounds.maxLon = maxLon + lonPad
        targetBounds.minLat = minLat - latPad
        targetBounds.maxLat = maxLat + latPad

        // FIX: Snap instantly on first load so we don't start at 0,0
        if (!isInitialized || (currentBounds.minLon === 0 && currentBounds.maxLon === 0)) {
            Object.assign(currentBounds, targetBounds)
            isInitialized = true
            // Force immediate D3 sync
            if (svgSelect) syncD3ToCamera()
        }
    }
}

// ----------------------------------------------------
// 2. ZOOM LOGIC
// ----------------------------------------------------
const setupZoom = () => {
    if (!svgSelect) return

    // Reference Scale: World Coords -> Arbitrary Pixel Space
    const refXScale = d3.scaleLinear().domain([-180, 180]).range([0, 360000])
    const refYScale = d3.scaleLinear().domain([-90, 90]).range([180000, 0])

    zoomBehavior = d3.zoom<SVGSVGElement, unknown>()
        .scaleExtent([0.1, 100000]) // Allow deep zoom
        .on('zoom', (event) => {
            if (event.sourceEvent) {
                isAutoMode.value = false

                const t = event.transform
                const newX = t.rescaleX(refXScale)
                const newY = t.rescaleY(refYScale)

                const w = containerWidth.value
                const h = containerHeight.value

                currentBounds.minLon = newX.invert(0)
                currentBounds.maxLon = newX.invert(w)
                currentBounds.maxLat = newY.invert(0)
                currentBounds.minLat = newY.invert(h)
            }
        })

    svgSelect.call(zoomBehavior).on("dblclick.zoom", null)
}

const syncD3ToCamera = () => {
    if (!svgSelect || !zoomBehavior) return

    const w = containerWidth.value
    const h = containerHeight.value

    const refXScale = d3.scaleLinear().domain([-180, 180]).range([0, 360000])
    const refYScale = d3.scaleLinear().domain([-90, 90]).range([180000, 0])

    // Calculate Transform that creates currentBounds
    const kx = w / (refXScale(currentBounds.maxLon) - refXScale(currentBounds.minLon))
    const ky = h / (refYScale(currentBounds.minLat) - refYScale(currentBounds.maxLat))
    const k = Math.min(kx, ky)

    const centerLon = (currentBounds.minLon + currentBounds.maxLon) / 2
    const centerLat = (currentBounds.minLat + currentBounds.maxLat) / 2

    const tx = (w / 2) - refXScale(centerLon) * k
    const ty = (h / 2) - refYScale(centerLat) * k

    const transform = d3.zoomIdentity.translate(tx, ty).scale(k)

    svgSelect.on("zoom", null)
    svgSelect.call(zoomBehavior.transform, transform)
    svgSelect.on("zoom", zoomBehavior.on("zoom"))
}

const toggleAutoMode = () => {
    isAutoMode.value = true
    isInitialized = false // Triggers snap/lerp logic in updateData
    updateData() // Force update
}

// ----------------------------------------------------
// 3. RENDER LOOP (High Frequency)
// ----------------------------------------------------
const renderLoop = () => {
    if (!svgSelect) return

    // A. AUTO MODE LERP
    if (isAutoMode.value && isInitialized) {
        const lerp = 0.1
        currentBounds.minLon += (targetBounds.minLon - currentBounds.minLon) * lerp
        currentBounds.maxLon += (targetBounds.maxLon - currentBounds.maxLon) * lerp
        currentBounds.minLat += (targetBounds.minLat - currentBounds.minLat) * lerp
        currentBounds.maxLat += (targetBounds.maxLat - currentBounds.maxLat) * lerp

        syncD3ToCamera()
    }

    // B. SCALES
    const w = containerWidth.value
    const h = containerHeight.value
    const padding = 0

    // Safety: Prevent 0 or NaN scales
    if (currentBounds.minLon === currentBounds.maxLon) {
        currentBounds.minLon -= 0.001
        currentBounds.maxLon += 0.001
    }

    const xScale = d3.scaleLinear().domain([currentBounds.minLon, currentBounds.maxLon]).range([padding, w - padding])
    const yScale = d3.scaleLinear().domain([currentBounds.minLat, currentBounds.maxLat]).range([h - padding, padding])

    // C. DRAW GRID
    const xAxis = d3.axisBottom(xScale).ticks(5).tickSize(-h).tickFormat(() => "")
    const yAxis = d3.axisLeft(yScale).ticks(5).tickSize(-w).tickFormat(() => "")

    // Using simple call here. We assume gGrid exists.
    gGrid.select<SVGGElement>('.x-grid').attr('transform', `translate(0, ${h})`).call(xAxis)
    gGrid.select<SVGGElement>('.y-grid').attr('transform', `translate(0, 0)`).call(yAxis)

    // D. DRAW LINKS
    const linksData: any[] = []
    const activeDrones = robots.value.filter(d => !d.is_stale)
    const processedPairs = new Set<string>()

    activeDrones.forEach(src => {
        if (!src.connectivity) return
        Object.entries(src.connectivity).forEach(([targetNid, strength]) => {
            const tgt = activeDrones.find(r => r.nid === targetNid)
            if (tgt && tgt.point && (strength as number) > 0) {
                const id = src.nid < tgt.nid ? `${src.nid}-${tgt.nid}` : `${tgt.nid}-${src.nid}`
                if (!processedPairs.has(id)) {
                    linksData.push({ id, src, tgt, strength })
                    processedPairs.add(id)
                }
            }
        })
    })

    const links = gContent.select('.links-group').selectAll('line').data(linksData, (d: any) => d.id)
    links.enter().append('line').attr('stroke-width', 2).attr('stroke-opacity', 0.6)
    links.exit().remove()
    links.merge(links.enter() as any)
        .attr('x1', d => xScale(d.src.point!.lon))
        .attr('y1', d => yScale(d.src.point!.lat))
        .attr('x2', d => xScale(d.tgt.point!.lon))
        .attr('y2', d => yScale(d.tgt.point!.lat))
        .attr('stroke', d => linkColorScale(d.strength))

    // E. DRAW NODES
    const nodes = gContent.select('.nodes-group').selectAll('g.node').data(robots.value, (d: any) => d.nid)
    const nodesEnter = nodes.enter().append('g').attr('class', 'node')
    nodesEnter.append('circle').attr('r', 6).attr('stroke', '#fff').attr('stroke-width', 2)
    nodesEnter.append('path').attr('d', 'M-4-4L4 4M4-4L-4 4').attr('stroke', 'red').attr('stroke-width', 2).attr('class', 'cross')
    nodesEnter.append('text').attr('y', -10).attr('text-anchor', 'middle').attr('fill', 'white').style('font-size', '10px').style('pointer-events', 'none')
        .style('text-shadow', '0 1px 2px black')

    nodes.exit().remove()
    const nodesMerge = nodes.merge(nodesEnter as any)
    nodesMerge.attr('transform', d => `translate(${xScale(d.point!.lon)}, ${yScale(d.point!.lat)})`)
    nodesMerge.select('circle')
        .attr('fill', d => getNodeColor(d.time_since_last_heard))
        .attr('stroke', d => d.is_stale ? '#444' : '#fff')
    nodesMerge.select('.cross').attr('opacity', d => d.is_stale ? 1 : 0)
    nodesMerge.select('text').text(d => d.display_name || d.nid.slice(-3))

    animationFrameId = requestAnimationFrame(renderLoop)
}

// ----------------------------------------------------
// LIFECYCLE
// ----------------------------------------------------
onMounted(() => {
    if (!svgRef.value) return

    svgSelect = d3.select(svgRef.value)

    // Grid Layer (Behind)
    gGrid = svgSelect.append('g').attr('class', 'layer-grid')
    gGrid.append('g').attr('class', 'x-grid')
    gGrid.append('g').attr('class', 'y-grid')

    // Content Layer
    gContent = svgSelect.append('g').attr('class', 'layer-content')
    gContent.append('g').attr('class', 'links-group')
    gContent.append('g').attr('class', 'nodes-group')

    setupZoom()

    updateData()
    dataInterval = setInterval(updateData, 200)
    renderLoop()

    if (containerRef.value) {
        resizeObserver = new ResizeObserver(entries => {
            containerWidth.value = entries[0].contentRect.width
            containerHeight.value = entries[0].contentRect.height
        })
        resizeObserver.observe(containerRef.value)
    }
})

onUnmounted(() => {
    if (dataInterval) clearInterval(dataInterval)
    if (animationFrameId) cancelAnimationFrame(animationFrameId)
    resizeObserver?.disconnect()
})
</script>

<template>
    <div class="radar-container" ref="containerRef">
        <svg ref="svgRef" width="100%" height="100%"></svg>

        <div class="controls">
            <button class="btn-control" :class="{ active: isAutoMode }" @click="toggleAutoMode"
                title="Re-center / Auto Follow">
                <OhVueIcon name="bi-crosshair" scale="1.2" />
                <span v-if="!isAutoMode" class="ms-1 text-xs">Re-center</span>
            </button>
        </div>
    </div>
</template>

<style scoped>
.radar-container {
    width: 100%;
    height: 100%;
    min-height: 200px;
    background: radial-gradient(circle, #222 0%, #111 100%);
    border-radius: 8px;
    border: 1px solid rgba(255, 255, 255, 0.1);
    overflow: hidden;
    position: relative;
    cursor: grab;
}

.radar-container:active {
    cursor: grabbing;
}

.controls {
    position: absolute;
    bottom: 10px;
    right: 10px;
    display: flex;
    gap: 5px;
}

.btn-control {
    background: rgba(0, 0, 0, 0.5);
    border: 1px solid rgba(255, 255, 255, 0.2);
    color: #fff;
    border-radius: 4px;
    padding: 4px 8px;
    display: flex;
    align-items: center;
    cursor: pointer;
    transition: all 0.2s;
}

.btn-control:hover {
    background: rgba(255, 255, 255, 0.1);
}

.btn-control.active {
    background: rgba(0, 255, 0, 0.2);
    border-color: rgba(0, 255, 0, 0.5);
    color: #4aff4a;
}
</style>

<style>
/* Global D3 Styles */
.layer-grid .domain {
    display: none;
}

.layer-grid line {
    stroke: rgba(255, 255, 255, 0.1);
    stroke-dasharray: 4;
}
</style>