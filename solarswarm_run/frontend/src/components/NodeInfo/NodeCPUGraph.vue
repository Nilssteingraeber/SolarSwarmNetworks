<script setup lang="ts">
import { onMounted, ref, watch, nextTick, onUnmounted } from 'vue'
import { Chart, LineController, LineElement, PointElement, LinearScale, Title, CategoryScale } from 'chart.js'
import { MDBCol, MDBRow } from 'mdb-vue-ui-kit'
import { UseViewedDroneStore } from '../../stores/viewedDroneStore'
import { useDroneHistoryStore } from '../../DronesData/DroneHistoryStore'
import { useTimeStore } from '../../stores/TimeStore'

const props = defineProps<{
    title: string
    dataFieldName: string
}>()

Chart.register(LineController, LineElement, PointElement, LinearScale, Title, CategoryScale)

const viewedDroneStore = UseViewedDroneStore()
const historyStore = useDroneHistoryStore()
const timeStore = useTimeStore()

const chartRef = ref<HTMLCanvasElement | null>(null)
let cpuChart: Chart<"line", any[], any> | null = null
let updateTimer: number | null = null

// Fetch full 60s window: left 30s = past, right 30s = future
async function refreshChartData() {
    const nid = viewedDroneStore.viewedNid
    if (!nid) return

    try {
        const currentTime = timeStore.currentTime

        await historyStore.ensureDroneDataLoaded(nid, currentTime, 60_000)

        const nidMap = historyStore.cache.get(nid)
        if (!nidMap) return

        // Rest of your existing chart data logic...
        const halfWindowMs = 30 * 1000
        const pastStart = currentTime - halfWindowMs
        const futureEnd = currentTime + halfWindowMs

        const allEntries = Array.from(nidMap.entries())
            .filter(([ts]) => ts >= pastStart && ts <= futureEnd)
            .sort(([a], [b]) => a - b)

        // Your existing data filling logic...
    } catch (error) {
        console.error('Chart refresh failed:', error)
    }
}



onMounted(async () => {
    await nextTick()

    const ctx = chartRef.value!.getContext('2d')!

    // Your existing plugins (unchanged)
    Chart.register({
        id: 'lineShadow',
        beforeDatasetDraw(chart, args) {
            const { ctx } = chart
            const dataset = chart.data.datasets[args.index]
            if (dataset.type === 'line' || chart.config.type === 'line') {
                ctx.save()
                ctx.shadowColor = 'rgba(0, 0, 0, 0.35)'
                const dynamicBlur = Math.min(chart.height * 0.02, 8)
                ctx.shadowBlur = dynamicBlur
                const dynamicOffset = Math.min(chart.height * 0.01, 4)
                ctx.shadowOffsetX = 0
                ctx.shadowOffsetY = dynamicOffset
            }
        },
        afterDatasetDraw(chart) {
            chart.ctx.restore()
        }
    })

    const latestValueLabelPlugin = {
        id: 'latestValueLabel',
        afterDraw(chart: { ctx: any; chartArea: any; scales: any; data: any; }) {
            const { ctx, data } = chart
            const dataset = data.datasets[0]
            const index = 28   // center ("now") index
            const value = dataset.data[index]

            if (value == null) return

            // Get the point's pixel position
            const meta = chart.getDatasetMeta(0)
            const point = meta.data[index]
            if (!point) return

            const { x, y } = point.getProps(['x', 'y'], true)

            const textX = x
            const textY = y
            const text = Math.round(value)
            const color = '#4f4f4f'
            const shadowColor = '#e2e2e2c5'
            ctx.fillStyle = shadowColor
            ctx.fillText(text, textX - 1, textY - 1)
            ctx.fillText(text, textX + 1, textY - 1)
            ctx.fillText(text, textX - 1, textY + 1)
            ctx.fillText(text, textX + 1, textY + 1)
            ctx.fillStyle = color
            ctx.fillText(text, textX, textY)
            ctx.restore()
        }
    }

    const leftFadeMaskPlugin = {
        id: 'leftFadeMask',
        afterDraw(chart: { ctx: any; canvas: any; }) {
            const { ctx, canvas } = chart
            ctx.save()
            const gradient = ctx.createLinearGradient(canvas.width * 0.05, 0, canvas.width * 0.45, 0)
            gradient.addColorStop(1, 'rgba(0,0,0,0)')
            gradient.addColorStop(0.5, 'rgba(0,0,0,0)')
            gradient.addColorStop(0, 'rgba(0,0,0,1)')
            ctx.globalCompositeOperation = 'destination-out'
            ctx.fillStyle = gradient
            ctx.fillRect(0, 0, canvas.width, canvas.height)
            ctx.globalCompositeOperation = 'source-over'
            ctx.restore()
        }
    }
    Chart.register(leftFadeMaskPlugin)

    // Initial chart setup
    cpuChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: Array(60).fill(''),
            datasets: [{
                data: Array(60).fill(null),
                borderWidth: 2,
                tension: 0.25,
                pointRadius: 0,
                borderColor: 'green'
            }]
        },
        plugins: [leftFadeMaskPlugin, latestValueLabelPlugin],
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            scales: {
                y: {
                    beginAtZero: true,
                    max: 100,
                    ticks: { stepSize: 50, display: false },
                    grid: { drawTicks: false, drawOnChartArea: true },
                    display: true
                },
                x: { display: false }
            }
        }
    })

    // Start 5s refresh
    updateTimer = window.setInterval(refreshChartData, 1000)
    refreshChartData()
})

onUnmounted(() => {
    if (updateTimer) window.clearInterval(updateTimer)
    if (cpuChart) cpuChart.destroy()
})

// Watch drone & sim time changes
watch([
    () => viewedDroneStore.viewedNid,
], refreshChartData, { immediate: true })
</script>

<!-- Template unchanged -->


<template>
    <MDBRow class="mb-1 w-100 h-100">
        <MDBCol class="text-center status-bar-text-banner-small">
            {{ title }}
        </MDBCol>
        <MDBCol class="col-6 status-bar-text-banner-small graph-frame">
            <div class="rounded py-1 w-100 h-100 rel">
                <div class="graph-canvas">
                    <canvas ref="chartRef" class="w-100 h-100"></canvas>
                </div>
                <p class="graph-y-tick-100 white-text-outline">100</p>
                <p class="graph-y-tick-50 white-text-outline">50</p>
                <p class="graph-y-tick-0 white-text-outline">0</p>
            </div>
        </MDBCol>
    </MDBRow>
</template>

<style scoped>
.rel {
    position: relative;
}

.white-text-outline {
    text-shadow: -1px -1px 0 #e2e2e2c5, 1px -1px 0 #e2e2e2c5, -1px 1px 0 #e2e2e2c5, 1px 1px 0 #e2e2e2c5;
}

.graph-y-tick-100 {
    position: absolute;
    top: 0%;
    transform: translateY(-23%);
    z-index: 1;
}

.graph-y-tick-50 {
    position: absolute;
    top: 52.5%;
    transform: translateY(-50%);
    z-index: 1;
}

.graph-y-tick-0 {
    position: absolute;
    top: 89%;
    z-index: 1;
}

.graph-canvas {
    width: 100%;
    height: 100%;
}

.graph-frame {
    position: relative;
    width: 100%;
    height: 100%;
    border-radius: 0.375rem;
}

.status-bar-text-banner-small {
    font-size: 12px;
    font-weight: bold;
}

.center-arrow {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    font-size: 12px;
    font-weight: bold;
    color: #ffffff;
    text-shadow:
        -1px -1px 2px rgba(0, 0, 0, 0.8),
        1px -1px 2px rgba(0, 0, 0, 0.8),
        -1px 1px 2px rgba(0, 0, 0, 0.8),
        1px 1px 2px rgba(0, 0, 0, 0.8);
    z-index: 10;
    pointer-events: none;
    /* so it never blocks mouse interactions */
}
</style>
