<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'
import * as d3 from 'd3'
import { RobotHistoryEntry, useDroneHistoryStore } from '../DronesData/DroneHistoryStore'
import type { Robot } from '../models/Robot'
import { useTimeStore } from '../stores/TimeStore'

/** Extended robot info for visualization */
interface ProcessedRobot extends Robot {
    time_since_last_heard: number
    is_stale: boolean
}

/** Props: optional width/height */
const props = defineProps<{ width?: number; height?: number }>()

/** Stores */
const historyStore = useDroneHistoryStore()
const timeStore = useTimeStore() // reactive time

/** DOM refs */
const containerRef = ref<HTMLDivElement | null>(null)
const svgRef = ref<SVGSVGElement | null>(null)

/** Container size */
const containerWidth = ref(props.width || 400)
const containerHeight = ref(props.height || 400)
let resizeObserver: ResizeObserver | null = null

/** Visualization constants */
const TRANSITION_DURATION = 500
const THRESHOLD_STALE = 7 // seconds
let timer: ReturnType<typeof setInterval> | null = null

/** Reactive list of processed robots */
const robots = ref<ProcessedRobot[]>([])

/** Node color based on freshness */
const getNodeColor = (timeSinceLastSeen: number): string => {
    if (timeSinceLastSeen < 3) return '#00FF00'
    if (timeSinceLastSeen < 5) return '#FFFF00'
    if (timeSinceLastSeen < 7) return '#FF0000'
    return '#888888'
}

/** Alternative D3 color scale for links */
const colorScale = d3.scaleLinear<string>()
    .domain([0, 0.5, 1])
    .range(['#FF0000', '#FFFF00', '#00FF00'])
    .clamp(true)

// ----------------------------------------------------
// UPDATE ROBOTS: compute latest snapshot for each drone
// ----------------------------------------------------
const updateRobots = async () => {
    const currentTime = timeStore.currentTime
    const snapshot: ProcessedRobot[] = []
    const windowMs = 2500
    const THRESHOLD_STALE = 5 // example value

    const allNids = Array.from(historyStore.cache.keys())

    // Parallel load to avoid sequential await lag
    await Promise.all(allNids.map(nid => historyStore.ensureDroneDataLoaded(nid, currentTime, windowMs)))

    for (const nid of allNids) {
        const entry = await historyStore.getSnapshotAt(nid, currentTime, windowMs)

        if (!entry) continue

        // Logic is now clean: entry.timestamp is the exact time we "heard" the robot
        const timeSinceLastHeard = (currentTime - entry.timestamp) / 1000

        snapshot.push({
            ...entry.data,
            time_since_last_heard: timeSinceLastHeard,
            is_stale: timeSinceLastHeard >= THRESHOLD_STALE,
        })
    }

    robots.value = snapshot
    requestAnimationFrame(draw)
}



// ----------------------------------------------------
// DRAW FUNCTION: draw nodes + links using D3
// ----------------------------------------------------
const draw = () => {
    const data = robots.value
    if (!svgRef.value || data.length === 0) return

    const w = containerWidth.value
    const h = containerHeight.value
    const padding = 40

    const svg = d3.select(svgRef.value).attr('width', w).attr('height', h)
    let rootGroup = svg.select('g.root-group')
    if (rootGroup.empty()) rootGroup = svg.append('g').attr('class', 'root-group')

    // Compute bounds
    const lons = data.map(d => d.point!.lon)
    const lats = data.map(d => d.point!.lat)
    let minLon = Math.min(...lons), maxLon = Math.max(...lons)
    let minLat = Math.min(...lats), maxLat = Math.max(...lats)

    // Add small buffer
    const lonBuffer = (maxLon - minLon) * 0.15
    const latBuffer = (maxLat - minLat) * 0.15
    minLon -= lonBuffer; maxLon += lonBuffer
    minLat -= latBuffer; maxLat += latBuffer
    if (maxLon - minLon < 0.0001) { minLon -= 0.00005; maxLon += 0.00005 }
    if (maxLat - minLat < 0.0001) { minLat -= 0.00005; maxLat += 0.00005 }

    // Scales
    const xScale = d3.scaleLinear().domain([minLon, maxLon]).range([padding, w - padding])
    const yScale = d3.scaleLinear().domain([minLat, maxLat]).range([h - padding, padding])

    // -------------------------
    // DRAW CONNECTIONS / LINKS
    // -------------------------
    const linksData: any[] = []
    const processedPairs = new Set<string>()
    const activeDrones = data.filter(d => !d.is_stale)

    activeDrones.forEach(source => {
        if (!source.connectivity || !source.point) return
        Object.entries(source.connectivity).forEach(([targetNid, strength]) => {
            const target = activeDrones.find(r => r.nid === targetNid)
            if (target && target.point && (strength as number) > 0) {
                const idA = source.nid
                const idB = target.nid
                const canon = idA < idB ? `${idA}--${idB}` : `${idB}--${idA}`
                if (!processedPairs.has(canon)) {
                    linksData.push({ id: canon, source, target, strength: strength as number })
                    processedPairs.add(canon)
                }
            }
        })
    })

    let linksGroup = rootGroup.select('g.links')
    if (linksGroup.empty()) linksGroup = rootGroup.append('g').attr('class', 'links')

    const allLinks = linksGroup.selectAll('line.connection-line').data(linksData, d => d.id)
    allLinks.exit().transition().duration(TRANSITION_DURATION).attr('stroke-opacity', 0).remove()

    const enteringLinks = allLinks.enter()
        .append('line')
        .attr('class', 'connection-line')
        .attr('stroke', d => colorScale(d.strength))
        .attr('stroke-width', d => 1 + d.strength * 2)
        .attr('stroke-opacity', 0)
        .attr('x1', d => xScale(d.source.point!.lon))
        .attr('y1', d => yScale(d.source.point!.lat))
        .attr('x2', d => xScale(d.target.point!.lon))
        .attr('y2', d => yScale(d.target.point!.lat))

    enteringLinks.merge(allLinks as any)
        .transition().duration(TRANSITION_DURATION)
        .attr('stroke', d => colorScale(d.strength))
        .attr('stroke-width', d => 1 + d.strength * 2)
        .attr('stroke-opacity', d => 0.6 + 0.4 * d.strength)
        .attr('x1', d => xScale(d.source.point!.lon))
        .attr('y1', d => yScale(d.source.point!.lat))
        .attr('x2', d => xScale(d.target.point!.lon))
        .attr('y2', d => yScale(d.target.point!.lat))

    // -------------------------
    // DRAW NODES
    // -------------------------
    let nodesGroup = rootGroup.select('g.nodes')
    if (nodesGroup.empty()) nodesGroup = rootGroup.append('g').attr('class', 'nodes')

    const nodes = nodesGroup.selectAll('g.node').data(data, d => d.nid)
    nodes.exit().transition().duration(TRANSITION_DURATION)
        .attr('transform', d => `translate(${xScale(d.point!.lon)}, ${yScale(d.point!.lat)}) scale(0)`)
        .remove()

    const enteringNodes = nodes.enter()
        .append('g')
        .attr('class', 'node')
        .attr('transform', d => `translate(${xScale(d.point!.lon)}, ${yScale(d.point!.lat)}) scale(0)`)

    enteringNodes.append('circle')
        .attr('r', 6)
        .attr('stroke', '#fff')
        .attr('stroke-width', 2)
        .attr('class', 'main-circle')

    enteringNodes.append('line')
        .attr('class', 'cross-out cross-out-1')
        .attr('x1', -5).attr('y1', -5)
        .attr('x2', 5).attr('y2', 5)
        .attr('stroke', '#FF0000').attr('stroke-width', 2)
        .attr('opacity', 0)

    enteringNodes.append('line')
        .attr('class', 'cross-out cross-out-2')
        .attr('x1', 5).attr('y1', -5)
        .attr('x2', -5).attr('y2', 5)
        .attr('stroke', '#FF0000').attr('stroke-width', 2)
        .attr('opacity', 0)

    enteringNodes.append('text')
        .text(d => d.display_name || d.nid.substring(0, 4))
        .attr('y', -10)
        .attr('text-anchor', 'middle')
        .attr('fill', '#fff')
        .style('font-size', '10px')
        .style('pointer-events', 'none')

    const mergedNodes = enteringNodes.merge(nodes as any)
        .transition().duration(TRANSITION_DURATION)
        .attr('transform', d => `translate(${xScale(d.point!.lon)}, ${yScale(d.point!.lat)}) scale(1)`)

    mergedNodes.select('circle.main-circle')
        .transition().duration(TRANSITION_DURATION)
        .attr('fill', d => getNodeColor(d.time_since_last_heard))
        .attr('stroke', d => d.is_stale ? '#888' : '#fff')

    mergedNodes.selectAll('line.cross-out')
        .transition().duration(TRANSITION_DURATION)
        .attr('opacity', d => d.is_stale ? 1 : 0)
}

// ----------------------------------------------------
// LIFECYCLE
// ----------------------------------------------------
onMounted(() => {
    updateRobots()
    timer = setInterval(updateRobots, 1000)

    if (containerRef.value) {
        resizeObserver = new ResizeObserver(entries => {
            const entry = entries[0]
            containerWidth.value = entry.contentRect.width
            containerHeight.value = entry.contentRect.height
            draw()
        })
        resizeObserver.observe(containerRef.value)
    }
})

onUnmounted(() => {
    if (timer) clearInterval(timer)
    resizeObserver?.disconnect()
})
</script>

<template>
    <div class="radar-container" ref="containerRef">
        <svg ref="svgRef"></svg>
    </div>
</template>

<style scoped>
.radar-container {
    width: 100%;
    height: 100%;
    position: relative;
    min-height: 200px;
    background: rgba(30, 30, 30, 0.75);
    border: 1px solid rgba(255, 255, 255, 0.1);
    backdrop-filter: blur(6px);
    -webkit-backdrop-filter: blur(6px);
}
</style>
