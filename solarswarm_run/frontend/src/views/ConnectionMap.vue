<script setup lang="ts">
import { ref, onMounted, onUnmounted, shallowRef, computed } from 'vue'
import * as d3 from 'd3'
import { useDroneHistoryStore } from '../DronesData/DroneHistoryStore'
import type { Robot } from '../models/Robot'
import { useTimeStore } from '../stores/TimeStore'
import { OhVueIcon } from "oh-vue-icons";

interface ProcessedRobot extends Robot {
    lag_seconds: number
    status_color: string
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

// ----------------------------------------------------
// COLOR LOGIC (Synced with Cesium helper)
// ----------------------------------------------------
const linkColorScale = d3.scaleLinear<string>().domain([0, 0.5, 1]).range(['#FF4444', '#FFFF00', '#00FF00'])

const calculateStatusColor = (lagSeconds: number | null): string => {
    if (lagSeconds === null) return '#888888'; // Gray
    if (lagSeconds < 10) return '#00FF00';     // Green
    if (lagSeconds < 15) return '#FFFF00';     // Yellow
    if (lagSeconds < 25) return '#FF0000';     // Red
    return '#888888';                          // Gray (Stale)
}

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

        // --- Match Cesium Lag Logic ---
        const history = historyStore.cache.get(robot.nid);
        let lagSeconds: number | null = null;

        if (history && history.length > 0) {
            const idx = historyStore.findLastIndexBefore(history, currentTime);
            if (idx !== -1) {
                const pastEntry = history[idx];
                const heardMs = pastEntry.timestamp;

                const nowMs = currentTime < 10000000000 ? currentTime * 1000 : currentTime;
                const actualHeardMs = heardMs < 10000000000 ? heardMs * 1000 : heardMs;

                lagSeconds = (nowMs - actualHeardMs) / 1000.0;
            }
        }

        const statusColor = calculateStatusColor(lagSeconds);

        snapshot.push({
            ...robot,
            lag_seconds: lagSeconds ?? 999,
            status_color: statusColor,
            is_stale: lagSeconds === null || lagSeconds > 25
        })

        if (Math.abs(robot.point.lon) > 0.001 && Math.abs(robot.point.lat) > 0.001) {
            if (robot.point.lon < minLon) minLon = robot.point.lon
            if (robot.point.lon > maxLon) maxLon = robot.point.lon
            if (robot.point.lat < minLat) minLat = robot.point.lat
            if (robot.point.lat > maxLat) maxLat = robot.point.lat
            hasValidPoints = true
        }
    })

    robots.value = snapshot

    if (hasValidPoints && isAutoMode.value) {
        if (maxLon === minLon) { minLon -= 0.0005; maxLon += 0.0005 }
        if (maxLat === minLat) { minLat -= 0.0005; maxLat += 0.0005 }

        const lonPad = (maxLon - minLon) * 0.2
        const latPad = (maxLat - minLat) * 0.2

        targetBounds.minLon = minLon - lonPad
        targetBounds.maxLon = maxLon + lonPad
        targetBounds.minLat = minLat - latPad
        targetBounds.maxLat = maxLat + latPad

        if (!isInitialized || (currentBounds.minLon === 0 && currentBounds.maxLon === 0)) {
            Object.assign(currentBounds, targetBounds)
            isInitialized = true
            if (svgSelect) syncD3ToCamera()
        }
    }
}

// ----------------------------------------------------
// 2. ZOOM LOGIC (Omitted unchanged parts for brevity...)
// ----------------------------------------------------
const setupZoom = () => {
    if (!svgSelect) return
    const refXScale = d3.scaleLinear().domain([-180, 180]).range([0, 360000])
    const refYScale = d3.scaleLinear().domain([-90, 90]).range([180000, 0])

    zoomBehavior = d3.zoom<SVGSVGElement, unknown>()
        .scaleExtent([0.1, 100000])
        .on('zoom', (event) => {
            if (event.sourceEvent) {
                isAutoMode.value = false
                const t = event.transform
                const newX = t.rescaleX(refXScale); const newY = t.rescaleY(refYScale)
                currentBounds.minLon = newX.invert(0); currentBounds.maxLon = newX.invert(containerWidth.value)
                currentBounds.maxLat = newY.invert(0); currentBounds.minLat = newY.invert(containerHeight.value)
            }
        })
    svgSelect.call(zoomBehavior).on("dblclick.zoom", null)
}

const syncD3ToCamera = () => {
    if (!svgSelect || !zoomBehavior) return
    const w = containerWidth.value; const h = containerHeight.value
    const refXScale = d3.scaleLinear().domain([-180, 180]).range([0, 360000])
    const refYScale = d3.scaleLinear().domain([-90, 90]).range([180000, 0])
    const kx = w / (refXScale(currentBounds.maxLon) - refXScale(currentBounds.minLon))
    const ky = h / (refYScale(currentBounds.minLat) - refYScale(currentBounds.maxLat))
    const k = Math.min(kx, ky)
    const tx = (w / 2) - refXScale((currentBounds.minLon + currentBounds.maxLon) / 2) * k
    const ty = (h / 2) - refYScale((currentBounds.minLat + currentBounds.maxLat) / 2) * k
    const transform = d3.zoomIdentity.translate(tx, ty).scale(k)
    // @ts-expect-error
    svgSelect.on("zoom", null); svgSelect.call(zoomBehavior.transform, transform); svgSelect.on("zoom", zoomBehavior.on("zoom"))
}

const toggleAutoMode = () => { isAutoMode.value = true; isInitialized = false; updateData() }

// ----------------------------------------------------
// 3. RENDER LOOP
// ----------------------------------------------------
const renderLoop = () => {
    if (!svgSelect) return

    if (isAutoMode.value && isInitialized) {
        const lerp = 0.1
        currentBounds.minLon += (targetBounds.minLon - currentBounds.minLon) * lerp
        currentBounds.maxLon += (targetBounds.maxLon - currentBounds.maxLon) * lerp
        currentBounds.minLat += (targetBounds.minLat - currentBounds.minLat) * lerp
        currentBounds.maxLat += (targetBounds.maxLat - currentBounds.maxLat) * lerp
        syncD3ToCamera()
    }

    const xScale = d3.scaleLinear().domain([currentBounds.minLon, currentBounds.maxLon]).range([0, containerWidth.value])
    const yScale = d3.scaleLinear().domain([currentBounds.minLat, currentBounds.maxLat]).range([containerHeight.value, 0])

    const xAxis = d3.axisBottom(xScale).ticks(5).tickSize(-containerHeight.value).tickFormat(() => "")
    const yAxis = d3.axisLeft(yScale).ticks(5).tickSize(-containerWidth.value).tickFormat(() => "")

    gGrid.select<SVGGElement>('.x-grid').attr('transform', `translate(0, ${containerHeight.value})`).call(xAxis)
    gGrid.select<SVGGElement>('.y-grid').call(yAxis)

    // LINKS
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

    // NODES
    const nodes = gContent.select('.nodes-group').selectAll('g.node').data(robots.value, (d: any) => d.nid)
    const nodesEnter = nodes.enter().append('g').attr('class', 'node')
    nodesEnter.append('circle').attr('r', 6).attr('stroke-width', 2)
    nodesEnter.append('path').attr('d', 'M-4-4L4 4M4-4L-4 4').attr('stroke', 'red').attr('stroke-width', 2).attr('class', 'cross')
    nodesEnter.append('text').attr('y', -10).attr('text-anchor', 'middle').attr('fill', 'white').style('font-size', '10px').style('pointer-events', 'none').style('text-shadow', '0 1px 2px black')

    nodes.exit().remove()
    const nodesMerge = nodes.merge(nodesEnter as any)
    nodesMerge.attr('transform', d => `translate(${xScale(d.point!.lon)}, ${yScale(d.point!.lat)})`)
    nodesMerge.select('circle')
        .attr('fill', d => d.status_color)
        .attr('stroke', d => d.is_stale ? '#444' : '#fff')
    nodesMerge.select('.cross').attr('opacity', d => d.is_stale ? 1 : 0)
    nodesMerge.select('text').text(d => d.display_name || d.nid.slice(-3))

    animationFrameId = requestAnimationFrame(renderLoop)
}

onMounted(() => {
    if (!svgRef.value) return
    svgSelect = d3.select(svgRef.value)
    gGrid = svgSelect.append('g').attr('class', 'layer-grid')
    gGrid.append('g').attr('class', 'x-grid'); gGrid.append('g').attr('class', 'y-grid')
    gContent = svgSelect.append('g').attr('class', 'layer-content')
    gContent.append('g').attr('class', 'links-group'); gContent.append('g').attr('class', 'nodes-group')
    setupZoom(); updateData()
    dataInterval = setInterval(updateData, 200); renderLoop()
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