<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted, watch } from 'vue'
import * as Cesium from 'cesium'
import { OhVueIcon } from "oh-vue-icons"

// # --- Imports: Stores
import { useDroneEntityStore } from '../../dronesData/DroneEntityStore'
import { useTimeStore } from '../../stores/TimeStore'
import { useDroneHistoryStore } from '../../dronesData/DroneHistoryStore'

const droneEntityStore = useDroneEntityStore()
const droneHistoryStore = useDroneHistoryStore()
const timeStore = useTimeStore()

// # --- Config
const SPEED_OPTIONS = [0.5, 1, 2, 5, 10, 30, 60]
const TIME_STEPS = [
    1, 2, 5, 10, 15, 30,
    60, 120, 300, 600, 900, 1800,
    3600, 7200, 14400, 21600, 43200, 86400
]

// # --- State: UI & Refs
const isPlaying = ref(true)
const playbackSpeed = ref(1)
const currentTimeString = ref('')
const currentTimePct = ref(0)
const realTimePct = ref(-100)
const isTimelineLoading = ref(false)
const showDebug = ref(false)

const activityCanvas = ref<HTMLCanvasElement | null>(null)

// # --- State: Animation Physics (Lerp)
const targetDuration = ref(3600)
const visualDuration = ref(3600)
const targetCenter = ref<Cesium.JulianDate | null>(null)
const visualCenter = ref<Cesium.JulianDate | null>(null)

// # --- State: Caching
const lastFetchCenterUnix = ref(0)
const lastFetchDuration = ref(0)
let fetchTimeout: number | null = null
let animFrame: number | null = null

// # --- Utils & Helpers
const getClock = () => droneEntityStore.viewer?.clock

const isLive = () => {
    if (!droneEntityStore.viewer) return false
    const now = Cesium.JulianDate.now()
    const current = droneEntityStore.viewer.clock.currentTime
    const diff = Math.abs(Cesium.JulianDate.secondsDifference(now, current))
    return diff < 5.0
}

function findStartIndex(arr: number[], minVal: number): number {
    let left = 0, right = arr.length - 1, result = -1
    while (left <= right) {
        const mid = Math.floor((left + right) / 2)
        if (arr[mid] >= minVal) {
            result = mid
            right = mid - 1
        } else {
            left = mid + 1
        }
    }
    return result === -1 ? arr.length : result
}

const initWindow = () => {
    const clock = getClock()
    if (clock && !targetCenter.value) {
        const now = clock.currentTime.clone()
        targetCenter.value = now
        visualCenter.value = now.clone()
    }
}

// # --- Computed: Ticks
const currentStepSize = computed(() => {
    const idealStep = visualDuration.value / 16
    return TIME_STEPS.find(s => s >= idealStep) || 86400
})

const timestamps = computed(() => {
    if (!visualCenter.value) return []

    const step = currentStepSize.value
    const duration = visualDuration.value
    const halfDur = duration / 2

    const startWindow = Cesium.JulianDate.addSeconds(visualCenter.value, -halfDur, new Cesium.JulianDate())
    const startUnix = Cesium.JulianDate.toDate(startWindow).getTime() / 1000

    // Align to step grid
    const remainder = startUnix % step
    let currentUnix = (startUnix - remainder) - step
    const endUnix = (Cesium.JulianDate.toDate(Cesium.JulianDate.addSeconds(visualCenter.value, halfDur, new Cesium.JulianDate())).getTime() / 1000) + step

    const ticks = []

    while (currentUnix <= endUnix) {
        const date = new Date(currentUnix * 1000)
        const tickTime = Cesium.JulianDate.fromDate(date)
        const diff = Cesium.JulianDate.secondsDifference(tickTime, startWindow)
        const pct = (diff / duration) * 100

        let label = ''
        if (step < 60) label = date.toLocaleTimeString([], { minute: '2-digit', second: '2-digit' })
        else if (step >= 86400) label = date.toLocaleDateString([], { month: 'short', day: 'numeric' })
        else label = date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })

        ticks.push({ id: currentUnix, pct, label, isMajor: currentUnix % (step * 5) === 0 })
        currentUnix += step
    }
    return ticks
})

// # --- Logic: Data Fetching (Debounced)
watch([targetCenter, targetDuration], () => {
    if (fetchTimeout) clearTimeout(fetchTimeout);

    fetchTimeout = window.setTimeout(async () => {
        if (!targetCenter.value) return;

        const centerUnix = Cesium.JulianDate.toDate(targetCenter.value).getTime();
        const durationMs = targetDuration.value * 1000;

        // Don't refetch if current view is largely covered by last fetch
        const isCoverageGood =
            Math.abs(centerUnix - lastFetchCenterUnix.value) < (lastFetchDuration.value * 0.4) &&
            Math.abs(durationMs - lastFetchDuration.value) < (lastFetchDuration.value * 0.2);

        if (isCoverageGood && !isTimelineLoading.value) return;

        isTimelineLoading.value = true;
        const bufferMultiplier = 3;
        const fetchDuration = durationMs * bufferMultiplier;
        const halfFetch = fetchDuration / 2;

        const start = centerUnix - halfFetch;
        const end = centerUnix + halfFetch;

        try {
            await droneHistoryStore.loadActivityMarkers(start, end);
            lastFetchCenterUnix.value = centerUnix;
            lastFetchDuration.value = fetchDuration;
        } finally {
            isTimelineLoading.value = false;
        }
    }, 500);
});

// # --- Logic: Canvas Rendering
const drawCanvas = () => {
    const cvs = activityCanvas.value;
    if (!cvs || !visualCenter.value) return;

    const ctx = cvs.getContext('2d');
    if (!ctx) return;

    const width = cvs.clientWidth;
    const height = cvs.clientHeight;
    const dpr = window.devicePixelRatio || 1;

    // Handle high-DPI scaling
    if (cvs.width !== width * dpr || cvs.height !== height * dpr) {
        cvs.width = width * dpr;
        cvs.height = height * dpr;
        ctx.scale(dpr, dpr);
    }

    ctx.clearRect(0, 0, width, height);

    const centerMs = Cesium.JulianDate.toDate(visualCenter.value).getTime();
    const halfDurMs = (visualDuration.value / 2) * 1000;
    const startMs = centerMs - halfDurMs;
    const totalDurMs = visualDuration.value * 1000;

    const getX = (ms: number) => ((ms - startMs) / totalDurMs) * width;

    // 1. Draw Cached Regions (Green Zones)
    droneHistoryStore.cachedRanges.forEach(range => {
        const x = getX(range.start);
        const w = getX(range.end) - x;

        if (w > 0) {
            ctx.fillStyle = 'rgba(76, 175, 80, 0.35)';
            ctx.fillRect(x, 2, w, height - 4);
            ctx.strokeStyle = '#4caf50';
            ctx.lineWidth = 1;
            ctx.strokeRect(x, 2, w, height - 4);
        }
    });

    // 2. Draw Activity Markers (Dots)
    ctx.fillStyle = '#4caf50';
    ctx.globalAlpha = 0.8;

    droneHistoryStore.activityMarkers.forEach((timestamps) => {
        if (!timestamps || timestamps.length === 0) return;

        let i = findStartIndex(timestamps, startMs);
        const endMs = startMs + totalDurMs;
        let lastDrawnPx = -100;

        for (; i < timestamps.length; i++) {
            const ts = timestamps[i];
            if (ts > endMs) break;

            const px = getX(ts);
            // Simple LOD: skip if too close to last pixel
            if (px - lastDrawnPx < 4) continue;

            ctx.fillRect(px, height - 6, 2, 4);
            lastDrawnPx = px;
        }
    });
    ctx.globalAlpha = 1.0;
}

// # --- Logic: Animation Loop
const renderLoop = () => {
    const clock = getClock()
    if (!clock) {
        animFrame = requestAnimationFrame(renderLoop)
        return
    }
    if (!targetCenter.value || !visualCenter.value) {
        initWindow()
        animFrame = requestAnimationFrame(renderLoop)
        return
    }

    // 1. Smooth Zoom
    const lerp = 0.15
    if (Math.abs(targetDuration.value - visualDuration.value) > 0.1) {
        visualDuration.value += (targetDuration.value - visualDuration.value) * lerp
    } else {
        visualDuration.value = targetDuration.value
    }

    // 2. Smooth Pan
    const diff = Cesium.JulianDate.secondsDifference(targetCenter.value, visualCenter.value)
    if (Math.abs(diff) > 0.01) {
        visualCenter.value = Cesium.JulianDate.addSeconds(visualCenter.value, diff * lerp, new Cesium.JulianDate())
    } else {
        visualCenter.value = targetCenter.value.clone()
    }

    // 3. Update UI Props
    const halfDur = visualDuration.value / 2
    const startWindow = Cesium.JulianDate.addSeconds(visualCenter.value, -halfDur, new Cesium.JulianDate())

    const diffPlayhead = Cesium.JulianDate.secondsDifference(clock.currentTime, startWindow)
    currentTimePct.value = (diffPlayhead / visualDuration.value) * 100

    const now = Cesium.JulianDate.now()
    const diffNow = Cesium.JulianDate.secondsDifference(now, startWindow)
    realTimePct.value = (diffNow / visualDuration.value) * 100

    // 4. Auto-Pan Behavior
    if (isPlaying.value && currentTimePct.value > 95 && currentTimePct.value < 105) {
        panWindow(targetDuration.value * 0.25)
    }

    // 5. Sync State
    isPlaying.value = clock.shouldAnimate
    playbackSpeed.value = clock.multiplier

    const d = Cesium.JulianDate.toDate(clock.currentTime)
    currentTimeString.value = d.toLocaleString('en-GB', { hour: '2-digit', minute: '2-digit', second: '2-digit' })
    timeStore.setTime(d.getTime())

    drawCanvas()
    animFrame = requestAnimationFrame(renderLoop)
}

// # --- Interaction Handlers
const togglePlay = () => getClock()!.shouldAnimate = !getClock()!.shouldAnimate
const changeSpeed = () => {
    const c = getClock()!
    c.multiplier = SPEED_OPTIONS[(SPEED_OPTIONS.indexOf(playbackSpeed.value) + 1) % SPEED_OPTIONS.length]
}
const jumpToNow = () => {
    const c = getClock()!
    const now = Cesium.JulianDate.now()
    c.currentTime = now
    targetCenter.value = now.clone()
    c.shouldAnimate = true
}
const zoomIn = () => targetDuration.value = Math.max(30, targetDuration.value * 0.6)
const zoomOut = () => targetDuration.value = Math.min(172800, targetDuration.value * 1.4)

const panWindow = (secs: number) => {
    if (targetCenter.value)
        targetCenter.value = Cesium.JulianDate.addSeconds(targetCenter.value, secs, new Cesium.JulianDate())
}

const onScrub = (e: MouseEvent) => {
    const c = getClock()
    if (!c || !visualCenter.value) return
    const rect = (e.currentTarget as HTMLElement).getBoundingClientRect()
    const pct = (e.clientX - rect.left) / rect.width
    const start = Cesium.JulianDate.addSeconds(visualCenter.value, -visualDuration.value / 2, new Cesium.JulianDate())
    c.currentTime = Cesium.JulianDate.addSeconds(start, visualDuration.value * pct, new Cesium.JulianDate())
}

const onKeydown = (e: KeyboardEvent) => {
    if (e.key === 'ArrowLeft') panWindow(-targetDuration.value * 0.1);
    if (e.key === 'ArrowRight') panWindow(targetDuration.value * 0.1);
    if (e.code === 'Space') {
        e.preventDefault();
        togglePlay();
    }
}

// # --- Lifecycle
onMounted(() => {
    animFrame = requestAnimationFrame(renderLoop)
    window.addEventListener('keydown', onKeydown)
})

onUnmounted(() => {
    if (animFrame) cancelAnimationFrame(animFrame)
    window.removeEventListener('keydown', onKeydown)
})
</script>

<template>
    <div class="timeline-root glass-panel">

        <div class="toolbar ps-1 pe-2">
            <div class="row align-items-center g-0 h-100">

                <div class="col-4 d-flex align-items-center gap-2 toolbar-left">
                    <button class="icon-btn small" @click="togglePlay">
                        <OhVueIcon :name="isPlaying ? 'bi-pause-fill' : 'bi-play-fill'" scale="1.2" />
                    </button>
                    <div class="time-display font-mono">
                        {{ currentTimeString }}
                    </div>

                    <div class="d-flex align-items-center gap-2 ms-2 position-relative" @mouseenter="showDebug = true"
                        @mouseleave="showDebug = false">
                        <transition name="fade">
                            <div v-if="isTimelineLoading" class="loading-tag d-flex align-items-center gap-1">
                                <OhVueIcon name="io-refresh-circle-sharp" animation="spin" scale="0.8" />
                                <span class="loading-text">Loading...</span>
                            </div>
                        </transition>

                        <div class="loading-tag info d-flex align-items-center gap-1" style="cursor:help">
                            <OhVueIcon name="bi-info-circle-fill" scale="0.7" />
                            <span class="loading-text">{{ droneHistoryStore.activeRobotCount }} drones</span>
                        </div>

                        <transition name="fade">
                            <div v-if="showDebug" class="debug-tooltip">
                                <div class="debug-row"><strong>RAM Points:</strong> {{
                                    droneHistoryStore.debugStats.totalPointsInRam.toLocaleString() }}</div>
                                <div class="debug-row"><strong>Est. Memory:</strong> {{
                                    droneHistoryStore.debugStats.memoryUsageMb }} MB</div>
                                <div class="debug-row"><strong>DB Reads:</strong> {{
                                    droneHistoryStore.debugStats.dbReads }}</div>
                                <div class="debug-divider"></div>
                                <div class="debug-control">
                                    <label>Cache (s):</label>
                                    <input type="number" :value="droneHistoryStore.config.maxCacheWindowMs / 1000"
                                        @input="e => droneHistoryStore.config.maxCacheWindowMs = Number((e.target as HTMLInputElement).value) * 1000"
                                        style="width: 50px;" />
                                </div>
                            </div>
                        </transition>
                    </div>
                </div>

                <div class="col-4 d-flex justify-content-center align-items-center gap-1 toolbar-center">
                    <button class="icon-btn small" @click="panWindow(-targetDuration * 0.2)">
                        <OhVueIcon name="bi-chevron-left" />
                    </button>
                    <button class="icon-btn small" @click="zoomOut">
                        <OhVueIcon name="bi-dash" />
                    </button>
                    <span class="zoom-label mx-1">Zoom</span>
                    <button class="icon-btn small" @click="zoomIn">
                        <OhVueIcon name="bi-plus" />
                    </button>
                    <button class="icon-btn small" @click="panWindow(targetDuration * 0.2)">
                        <OhVueIcon name="bi-chevron-right" />
                    </button>
                </div>

                <div class="col-4 d-flex justify-content-end align-items-center gap-2 toolbar-right">
                    <button class="text-btn" @click="changeSpeed">{{ playbackSpeed }}x</button>
                    <button class="live-btn" :class="{ active: isLive() }" @click="jumpToNow">LIVE</button>
                </div>
            </div>
        </div>

        <div class="timeline-track-container" @click="onScrub">

            <canvas ref="activityCanvas" class="activity-layer"></canvas>

            <div class="ticks-layer">
                <div v-for="tick in timestamps" :key="tick.id" class="tick-mark" :style="{ left: tick.pct + '%' }">
                    <div class="tick-line" :class="{ major: tick.isMajor }"></div>
                    <div class="tick-label" :class="{ major: tick.isMajor }">{{ tick.label }}</div>
                </div>
            </div>

            <div class="now-marker" :style="{ left: realTimePct + '%' }">
                <div class="now-label">NOW</div>
            </div>

            <div class="playhead" :style="{ left: currentTimePct + '%' }">
                <div class="playhead-knob"></div>
                <div class="playhead-line"></div>
            </div>
        </div>
    </div>
</template>

<style scoped>
/* # --- Structural */
.timeline-root {
    position: fixed;
    bottom: 20px;
    left: 50%;
    transform: translateX(-50%);
    width: 80vw;
    height: 92px;
    z-index: 2000;
    background: rgba(255, 255, 255, 0.65);
    border-radius: 12px;
    border: 1px solid rgba(255, 255, 255, 0.5);
    box-shadow: 0 3px 6px rgba(0, 0, 0, 0.15);
    backdrop-filter: blur(16px) saturate(150%);
    display: flex;
    flex-direction: column;
    overflow: visible;
}

.toolbar {
    height: 42px;
    border-bottom: 1px solid rgba(136, 136, 136, 0.2);
}

.toolbar-left {
    min-width: 0;
}

.toolbar-center {
    pointer-events: auto;
}

.toolbar-right {
    white-space: nowrap;
}

.timeline-track-container {
    flex: 1;
    position: relative;
    cursor: crosshair;
    background: rgba(255, 255, 255, 0.2);
    padding: 6px 0;
    overflow: hidden;
    border-bottom-left-radius: 12px;
    border-bottom-right-radius: 12px;
}

/* # --- Ticks & Layers */
.ticks-layer {
    position: absolute;
    inset: 0;
    z-index: 3;
    pointer-events: none;
}

.tick-mark {
    position: absolute;
    top: 0;
    height: 100%;
}

.tick-line {
    width: 1px;
    height: 6px;
    background: rgba(0, 0, 0, 0.55);
}

.tick-line.major {
    height: 8px;
    background: rgba(0, 0, 0, 0.65);
}

.tick-label {
    position: absolute;
    top: 10px;
    left: 0px;
    font-size: 12px;
    font-family: monospace;
    color: #333;
    opacity: 0.6;
}

.tick-label.major {
    opacity: 1;
}

.activity-layer {
    position: absolute;
    inset: 0;
    width: 100%;
    height: 100%;
    pointer-events: none;
    z-index: 1;
}

/* # --- Markers */
.now-marker {
    position: absolute;
    top: 0;
    bottom: 0;
    border-left: 1px solid #007bff;
    z-index: 5;
    pointer-events: none;
}

.now-label {
    position: absolute;
    bottom: 2px;
    left: 2px;
    font-size: 8px;
    font-weight: bold;
    color: #007bff;
}

.playhead {
    position: absolute;
    top: 0;
    bottom: 0;
    width: 2px;
    z-index: 10;
    pointer-events: none;
}

.playhead-knob {
    position: absolute;
    top: -2px;
    left: -6px;
    width: 14px;
    height: 14px;
    background: #e63946;
    border-radius: 50%;
}

.playhead-line {
    position: absolute;
    top: 12px;
    bottom: 0;
    width: 2px;
    background: #e63946;
}

/* # --- Buttons & Controls */
.icon-btn {
    background: rgba(255, 255, 255, 0.5);
    border: 1px solid rgba(136, 136, 136, 0.3);
    border-radius: 6px;
    cursor: pointer;
    padding: 4px;
}

.icon-btn.small {
    padding: 3px;
}

.text-btn {
    background: transparent;
    border: 1px solid transparent;
    font-weight: bold;
    color: #555;
    cursor: pointer;
    padding: 2px 6px;
    border-radius: 4px;
}

.live-btn {
    border: 1px solid #ff4444;
    color: #ff4444;
    font-weight: bold;
    font-size: 11px;
    padding: 2px 6px;
    border-radius: 4px;
    cursor: pointer;
    transition: all 0.2s;
}

.live-btn:hover {
    background: #ff4444;
    color: white;
}

.live-btn.active {
    background: #ff4444;
    color: white;
    box-shadow: 0 0 8px rgba(255, 68, 68, 0.5);
    animation: pulse 2s infinite;
}

/* # --- Debug & Info */
.loading-tag {
    background: rgba(25, 118, 210, 0.1);
    padding: 2px 8px;
    border-radius: 20px;
    color: #1976d2;
}

.loading-tag.info {
    background: rgba(0, 0, 0, 0.05);
    color: #555;
}

.loading-text {
    font-size: 10px;
    font-weight: bold;
}

.debug-tooltip {
    position: absolute;
    bottom: 100%;
    left: 0;
    margin-bottom: 8px;
    background: rgba(0, 0, 0, 0.9);
    color: white;
    padding: 10px;
    border-radius: 6px;
    font-size: 11px;
    font-family: monospace;
    white-space: nowrap;
    z-index: 3000;
    pointer-events: auto;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.2);
}

.debug-row {
    margin-bottom: 2px;
}

.debug-divider {
    height: 1px;
    background: #555;
    margin: 6px 0;
}

.debug-control {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-bottom: 4px;
}

.debug-control label {
    width: 80px;
    text-align: right;
}

.debug-control input {
    background: #333;
    border: 1px solid #555;
    color: white;
    border-radius: 4px;
    padding: 1px 4px;
    font-size: 11px;
}

/* # --- Utils */
.font-mono {
    font-family: monospace;
    font-size: 14px;
    font-weight: bold;
    color: #333;
}

.fade-enter-active,
.fade-leave-active {
    transition: opacity 0.25s ease;
}

.fade-enter-from,
.fade-leave-to {
    opacity: 0;
}

@keyframes pulse {
    0% {
        box-shadow: 0 0 0 0 rgba(255, 68, 68, 0.7);
    }

    70% {
        box-shadow: 0 0 0 6px rgba(255, 68, 68, 0);
    }

    100% {
        box-shadow: 0 0 0 0 rgba(255, 68, 68, 0);
    }
}
</style>