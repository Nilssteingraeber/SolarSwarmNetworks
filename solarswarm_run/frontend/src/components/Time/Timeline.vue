<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import * as Cesium from 'cesium'
import { OhVueIcon } from "oh-vue-icons"
import { useDroneEntityStore } from '../../DronesData/DroneEntityStore'
import { useTimeStore } from '../../stores/TimeStore'

const droneEntityStore = useDroneEntityStore()
const timeStore = useTimeStore()

// --- STATE ---
const isPlaying = ref(true)
const playbackSpeed = ref(1)
const currentTimeString = ref('')

// Playhead Positions (0-100%)
const currentTimePct = ref(0)
const realTimePct = ref(-100) // Position of "Now"

// --- PHYSICS STATE (Lerp Targets) ---
const targetDuration = ref(3600)
const visualDuration = ref(3600)

const targetCenter = ref<Cesium.JulianDate | null>(null)
const visualCenter = ref<Cesium.JulianDate | null>(null)

const SPEED_OPTIONS = [0.5, 1, 2, 5, 10, 30, 60]

// Standard Time Steps for Ticks
const TIME_STEPS = [
    1, 2, 5, 10, 15, 30, // seconds
    60, 120, 300, 600, 900, 1800, // 1m, 2m, 5m...
    3600, 7200, 14400, 21600, 43200, 86400 // hours
]

const getClock = () => droneEntityStore.viewer?.clock

// ----------------------------------------------------------------
// INITIALIZATION
// ----------------------------------------------------------------
const initWindow = () => {
    const clock = getClock()
    if (clock && !targetCenter.value) {
        const now = clock.currentTime.clone()
        targetCenter.value = now
        visualCenter.value = now.clone()
    }
}

// ----------------------------------------------------------------
// SMART TICKS ENGINE
// ----------------------------------------------------------------

const currentStepSize = computed(() => {
    // Increased divisor to 16 to show MORE ticks (Denser)
    const idealStep = visualDuration.value / 16
    return TIME_STEPS.find(s => s >= idealStep) || 86400
})

const timestamps = computed(() => {
    if (!visualCenter.value) return []

    const step = currentStepSize.value
    const duration = visualDuration.value

    const halfDur = duration / 2
    const startWindow = Cesium.JulianDate.addSeconds(visualCenter.value, -halfDur, new Cesium.JulianDate())
    const endWindow = Cesium.JulianDate.addSeconds(visualCenter.value, halfDur, new Cesium.JulianDate())

    // Align first tick to grid
    const startUnix = Cesium.JulianDate.toDate(startWindow).getTime() / 1000
    const remainder = startUnix % step
    let currentUnix = (startUnix - remainder) - step // Start slightly before window

    const endUnix = (Cesium.JulianDate.toDate(endWindow).getTime() / 1000) + step
    const ticks = []

    while (currentUnix <= endUnix) {
        const date = new Date(currentUnix * 1000)

        // Calculate Position % based on Window Start
        const tickTime = Cesium.JulianDate.fromDate(date)
        const diff = Cesium.JulianDate.secondsDifference(tickTime, startWindow)
        const pct = (diff / duration) * 100

        // Label Logic
        let label = ''
        if (step < 60) {
            label = date.toLocaleTimeString([], { minute: '2-digit', second: '2-digit' })
        } else if (step >= 86400) {
            label = date.toLocaleDateString([], { month: 'short', day: 'numeric' })
        } else {
            label = date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
        }

        // Determine "Major" tick (e.g. every hour on the hour)
        const isMajor = currentUnix % (step * 5) === 0

        ticks.push({
            id: currentUnix,
            pct,
            label,
            isMajor
        })

        currentUnix += step
    }

    return ticks
})

// ----------------------------------------------------------------
// ANIMATION LOOP
// ----------------------------------------------------------------
let animFrame: number | null = null

const renderLoop = () => {
    const clock = getClock()
    if (!clock) {
        animFrame = requestAnimationFrame(renderLoop)
        return
    }
    if (!targetCenter.value) initWindow()

    // 1. PHYSICS (LERP)
    const lerp = 0.15
    if (Math.abs(targetDuration.value - visualDuration.value) > 0.1) {
        visualDuration.value += (targetDuration.value - visualDuration.value) * lerp
    } else {
        visualDuration.value = targetDuration.value
    }

    if (targetCenter.value && visualCenter.value) {
        const diff = Cesium.JulianDate.secondsDifference(targetCenter.value, visualCenter.value)
        if (Math.abs(diff) > 0.1) {
            visualCenter.value = Cesium.JulianDate.addSeconds(visualCenter.value, diff * lerp, new Cesium.JulianDate())
        } else {
            visualCenter.value = targetCenter.value.clone()
        }
    }

    // 2. CALCULATE POSITIONS (Playback & Realtime)
    const halfDur = visualDuration.value / 2
    const startWindow = Cesium.JulianDate.addSeconds(visualCenter.value!, -halfDur, new Cesium.JulianDate())

    // Playhead
    const diffPlayhead = Cesium.JulianDate.secondsDifference(clock.currentTime, startWindow)
    currentTimePct.value = (diffPlayhead / visualDuration.value) * 100

    // Real-Time Indicator ("NOW")
    const now = Cesium.JulianDate.now()
    const diffNow = Cesium.JulianDate.secondsDifference(now, startWindow)
    realTimePct.value = (diffNow / visualDuration.value) * 100

    // 3. AUTO SCROLL
    if (isPlaying.value && currentTimePct.value > 95) {
        panWindow(targetDuration.value * 0.25)
    }

    // 4. SYNC UI
    if (isPlaying.value !== clock.shouldAnimate) isPlaying.value = clock.shouldAnimate
    if (playbackSpeed.value !== clock.multiplier) playbackSpeed.value = clock.multiplier

    const d = Cesium.JulianDate.toDate(clock.currentTime)
    currentTimeString.value = d.toLocaleString('en-GB', { hour: '2-digit', minute: '2-digit', second: '2-digit' })
    timeStore.setTime(d.getTime())

    animFrame = requestAnimationFrame(renderLoop)
}

// ----------------------------------------------------------------
// ACTIONS
// ----------------------------------------------------------------
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
    const dur = visualDuration.value
    const start = Cesium.JulianDate.addSeconds(visualCenter.value, -dur / 2, new Cesium.JulianDate())
    c.currentTime = Cesium.JulianDate.addSeconds(start, dur * pct, new Cesium.JulianDate())
}

onMounted(() => animFrame = requestAnimationFrame(renderLoop))
onUnmounted(() => animFrame && cancelAnimationFrame(animFrame))
</script>

<template>
    <div class="timeline-root glass-panel">
        <div class="toolbar d-flex justify-content-between align-items-center px-3">
            <div class="d-flex align-items-center gap-2">
                <button class="icon-btn" @click="togglePlay">
                    <OhVueIcon :name="isPlaying ? 'bi-pause-fill' : 'bi-play-fill'" scale="1.2" />
                </button>
                <div class="time-display font-mono ms-2">{{ currentTimeString }}</div>
            </div>
            <div class="d-flex align-items-center gap-1 nav-group">
                <button class="icon-btn small" @click="panWindow(-targetDuration * 0.2)">
                    <OhVueIcon name="bi-chevron-left" />
                </button>
                <button class="icon-btn small" @click="zoomOut">
                    <OhVueIcon name="bi-dash" />
                </button>
                <span class="zoom-label">Zoom</span>
                <button class="icon-btn small" @click="zoomIn">
                    <OhVueIcon name="bi-plus" />
                </button>
                <button class="icon-btn small" @click="panWindow(targetDuration * 0.2)">
                    <OhVueIcon name="bi-chevron-right" />
                </button>
            </div>
            <div class="d-flex align-items-center gap-2">
                <button class="text-btn" @click="changeSpeed">{{ playbackSpeed }}x</button>
                <button class="live-btn" @click="jumpToNow">LIVE</button>
            </div>
        </div>

        <div class="timeline-track-container" @click="onScrub">
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

            <div class="hover-highlight"></div>
        </div>
    </div>
</template>

<style scoped>
@font-face {
    font-family: 'Latin Modern Math';
    src: url("@/assets/latinmodern-math.otf") format('opentype');
}

.font-mono {
    font-family: monospace;
    font-size: 14px;
    font-weight: bold;
    color: #333;
}

.timeline-root {
    position: fixed;
    bottom: 20px;
    left: 50%;
    transform: translateX(-50%);
    width: 80vw;
    max-width: 90vw;
    height: 85px;
    z-index: 2000;
    background: rgba(255, 255, 255, 0.65);
    border-radius: 12px;
    border: 1px solid rgba(255, 255, 255, 0.5);
    box-shadow: 0 3px 6px rgba(0, 0, 0, 0.15);
    backdrop-filter: blur(16px) saturate(150%);
    display: flex;
    flex-direction: column;
    overflow: hidden;
    user-select: none;
}

.toolbar {
    height: 40px;
    border-bottom: 1px solid rgba(136, 136, 136, 0.2);
}

.timeline-track-container {
    flex: 1;
    position: relative;
    cursor: crosshair;
    background: rgba(255, 255, 255, 0.2);
    overflow: hidden;
}

/* Ticks */
.ticks-layer {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    pointer-events: none;
}

.tick-mark {
    position: absolute;
    top: 0;
    height: 100%;
    width: 1px;
}

.tick-line {
    width: 2px;
    height: 6px;
    background: rgba(0, 0, 0, 0.2);
}

.tick-line.major {
    height: 12px;
    background: rgba(0, 0, 0, 0.5);
    width: 2px;
}

.tick-label {
    position: absolute;
    top: 8px;
    left: 3px;
    font-size: 12px;
    color: #333;
    font-family: monospace;
    white-space: nowrap;
    font-weight: bold;
    opacity: 0.7;
}

.tick-label.major {
    top: 14px;
    font-size: 12px;
    color: #333;
    font-weight: bold;
    opacity: 1;
}

/* "NOW" Indicator (Blue) */
.now-marker {
    position: absolute;
    top: 0;
    bottom: 0;
    width: 1px;
    border-left: 1px dashed #007bff;
    /* Blue dashed line */
    z-index: 5;
    pointer-events: none;
    opacity: 0.7;
}

.now-label {
    position: absolute;
    bottom: 2px;
    left: 2px;
    font-size: 8px;
    font-weight: bold;
    color: #007bff;
    background: rgba(255, 255, 255, 0.8);
    padding: 0 2px;
    border-radius: 2px;
}

/* Playhead (Red) */
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
    box-shadow: 0 1px 3px rgba(0, 0, 0, 0.3);
}

.playhead-line {
    position: absolute;
    top: 12px;
    bottom: 0;
    left: 0;
    width: 2px;
    background: #e63946;
}

/* Buttons */
.icon-btn {
    background: rgba(255, 255, 255, 0.5);
    border: 1px solid rgba(136, 136, 136, 0.3);
    border-radius: 6px;
    color: #444;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    padding: 4px 8px;
    transition: all 0.3s;
}

.icon-btn:hover {
    background: #fff;
    color: #007bff;
}

.icon-btn.small {
    padding: 2px 6px;
}

.zoom-label {
    font-size: 10px;
    text-transform: uppercase;
    color: #666;
    margin: 0 4px;
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

.text-btn:hover {
    background: rgba(0, 0, 0, 0.05);
}

.live-btn {
    background: transparent;
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

.d-flex {
    display: flex;
}

.justify-content-between {
    justify-content: space-between;
}

.align-items-center {
    align-items: center;
}

.gap-1 {
    gap: 4px;
}

.gap-2 {
    gap: 8px;
}

.px-3 {
    padding-left: 1rem;
    padding-right: 1rem;
}

.py-1 {
    padding-top: 0.25rem;
    padding-bottom: 0.25rem;
}

.ms-2 {
    margin-left: 0.5rem;
}
</style>