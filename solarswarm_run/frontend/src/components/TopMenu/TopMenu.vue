<script setup lang="ts">
import { computed, ref, watch, onUnmounted, toRaw } from 'vue'
import { storeToRefs } from 'pinia'
import { degToRad } from 'three/src/math/MathUtils'
import {
    Viewer, ScreenSpaceEventHandler, Cartesian2, defined,
    ScreenSpaceEventType, HeadingPitchRange, Cartesian3,
    ConstantProperty
} from 'cesium'

// UI & Components
import { MDBRow, MDBCol } from 'mdb-vue-ui-kit'
import { OhVueIcon } from 'oh-vue-icons'
import NodeSideMenu from '../NodeInfo/NodeSideMenu.vue'
import GeoEditingMainMenu from '../GeoEditing/GeoEditingMainMenu.vue'

// Stores & Types
import { useDroneEntityStore } from '../../dronesData/DroneEntityStore'
import { useGeoToolsStore } from '../../stores/GeoToolsStore'
import { UseViewedDroneStore } from '../../stores/viewedDroneStore'
import { useSettingsStore } from '../../stores/SettingsStore'
import { useDroneHistoryStore } from '../../dronesData/DroneHistoryStore'
import { useTimeStore } from '../../stores/TimeStore'
import type { RobotWithEntity } from '../../models/Robot'

// # --- Init Stores ---
const viewedDroneStore = UseViewedDroneStore()
const { viewedRobot } = storeToRefs(viewedDroneStore)

const droneEntityStore = useDroneEntityStore()
const { viewer } = storeToRefs(droneEntityStore)

const geoStore = useGeoToolsStore()
const { isOpen: isGeoToolsOpen } = storeToRefs(geoStore)

const droneHistoryStore = useDroneHistoryStore()
const timeStore = useTimeStore()
const settingsStore = useSettingsStore()

// # --- State ---
const fileInput = ref<HTMLInputElement | null>(null)
const showDroneList = ref(false)
const isScrolledDown = ref(false)
const searchQuery = ref('')
let handler: ScreenSpaceEventHandler | null = null

// # --- Computed ---
const isAnyMenuOpen = computed(() => viewedRobot?.value || isGeoToolsOpen.value)
const isNodeMenuOpen = computed(() => viewedRobot?.value)
const allDrones = computed(() => droneEntityStore.getRobots ?? [])
const dronesAmount = computed(() => droneHistoryStore.knownNids.size)
const mapIconName = computed(() => settingsStore.getShow3dMesh() ? 'ri-map-2-fill' : 'ri-map-2-line')

const filteredDrones = computed(() => {
    const q = searchQuery.value.trim().toLowerCase()
    if (!q) return allDrones.value
    return allDrones.value.filter(d =>
        d.robot.display_name?.toLowerCase().includes(q) ||
        d.robot.nid.toLowerCase().includes(q)
    )
})

// # --- Methods: Data ---
const downloadData = () => droneHistoryStore.exportDatabaseToFile()

const triggerFilePicker = () => fileInput.value?.click()

const handleFileImport = async (event: Event) => {
    const target = event.target as HTMLInputElement
    const file = target.files?.[0]
    if (!file) return

    try {
        await droneHistoryStore.importDatabaseFromFile(file)
        alert('History data imported successfully!')
    } catch (err) {
        console.error('Import failed: ', err)
        alert('Failed to import file. Ensure it is a valid JSON history export.')
    } finally {
        target.value = ''
    }
}

// # --- Methods: View & UI ---
const toggleShow3dmesh = () => settingsStore.setShow3dMesh(!settingsStore.getShow3dMesh())

const toggleMapFocus = () => {
    const target = document.documentElement
    isScrolledDown.value = target.scrollTop < 10
    window.scrollTo({
        top: isScrolledDown.value ? (target.scrollHeight - target.clientHeight) : 0,
        behavior: 'smooth'
    })
}

const toggleOpenGeoTools = () => {
    geoStore.toggleOpen()
    if (geoStore.isOpen) {
        viewedDroneStore.setDrone(null)
        droneEntityStore.removeFlightPathLines(viewedDroneStore.viewedNid ?? '')
    }
    showDroneList.value = false
}

const getStatusColor = (nid: string) => {
    const history = droneHistoryStore.cache.get(nid)
    if (!history?.length) return 'gray'

    const currentTime = timeStore.currentTime
    const idx = droneHistoryStore.findLastIndexBefore(history, currentTime)
    if (idx === -1) return '#808080'

    const lagSeconds = (currentTime - (history[idx]?.data?.last_heard ?? Number.MAX_SAFE_INTEGER)) / 1000.0
    if (lagSeconds < 10) return '#008000'
    if (lagSeconds < 15) return '#ffff00'
    if (lagSeconds < 20) return '#ff0000'
    return '#808080'
}

// # --- Methods: Drone Interaction ---
const focusDrone = (nid: string) => {
    showDroneList.value = false
    searchQuery.value = ""

    const v = viewer.value
    const drone = droneEntityStore.getRobot(nid) as RobotWithEntity | undefined
    if (!v || !drone?.entity) return

    droneEntityStore.removeFlightPathLines(viewedDroneStore.viewedNid ?? '')
    viewedDroneStore.setDrone(null, nid)
    geoStore.setOpen(false)

    const range = 150
    const pitch = degToRad(-35)

    // Offset calculation for viewFrom
    const yOffset = -range * Math.cos(pitch)
    const zOffset = -range * Math.sin(pitch)
    const trackingOffset = new Cartesian3(0, yOffset, zOffset)

    drone.entity.viewFrom = new ConstantProperty(trackingOffset)
    v.flyTo(drone.entity, {
        duration: 1.5,
        offset: new HeadingPitchRange(0, pitch, range)
    }).then(() => {
        v.trackedEntity = toRaw(drone.entity)
    })
}

// # --- Cesium Selection Handler ---
function attachDroneSelector(v: Viewer) {
    detachDroneSelector()
    if (!v.scene?.canvas) return

    handler = new ScreenSpaceEventHandler(v.scene.canvas)
    handler.setInputAction((click: { position: Cartesian2 }) => {
        const picked = v.scene.pick(click.position)
        if (!defined(picked)) return

        const id = (picked as any).id ?? (picked as any).primitive?.id
        const prop = id?.properties?.droneData
        const nid = prop?.getValue?.()?.nid ?? prop?.nid
        if (nid) focusDrone(nid)
    }, ScreenSpaceEventType.LEFT_DOUBLE_CLICK)
}

function detachDroneSelector() {
    handler?.destroy()
    handler = null
}

// # --- Lifecycle ---
watch(viewer, v => v ? attachDroneSelector(v) : detachDroneSelector(), { immediate: true })
onUnmounted(detachDroneSelector)
</script>

<template>
    <MDBRow class="menu-root non-clickable">
        <MDBCol>
            <MDBRow>
                <MDBCol class="col-auto menu mx-2 d-flex align-items-center clickable">
                    <div class="title">Solar Swarm</div>
                </MDBCol>

                <MDBCol class="col-auto menu mx-3 d-flex flex-column align-items-center clickable">
                    <MDBRow>
                        <MDBCol class="input-group ps-1 align-items-center">
                            <div class="form-outline search-outline-fix">
                                <input type="text" id="searchForm" class="form-control text-input-layer"
                                    v-model="searchQuery" placeholder="Search drones…" @focus="showDroneList = true"
                                    @focusout="showDroneList = false" />
                            </div>
                            <button type="button" class="btn btn-primary search-button-fix me-2">
                                <OhVueIcon name="bi-search" scale="1" class="icon" />
                            </button>
                            <button type="button"
                                class="icon-button d-flex align-items-center p-0 dropdown-list cursor-pointer"
                                @click="showDroneList = !showDroneList">
                                <div class="p-2 title_small">{{ dronesAmount }}</div>
                                <OhVueIcon name="la-robot-solid" scale="1.5" class="icon icon-button p-1" />
                                <OhVueIcon :name="showDroneList ? 'bi-caret-down-fill' : 'bi-caret-up-fill'" scale="1"
                                    class="icon icon-button ps-0 pe-2" />
                            </button>
                        </MDBCol>
                    </MDBRow>

                    <transition name="expand-fade">
                        <MDBRow class="w-100 px-0" v-if="showDroneList">
                            <MDBCol class="drone-list px-2">
                                <div v-if="filteredDrones.length === 0" class="p-3 text-center">
                                    {{ searchQuery ? 'Keine Drohnen gefunden' : 'Lade Drohnen...' }}
                                </div>
                                <MDBRow v-for="d in [...filteredDrones]" :key="d.robot.nid"
                                    @click="focusDrone(d.robot.nid)"
                                    class="p-1 px-0 cursor-pointer drone-entry d-flex align-items-center input-layer my-2">
                                    <MDBCol class="col-auto px-0">
                                        <OhVueIcon name="la-robot-solid" scale="1"
                                            class="icon p-1 ps-2 d-flex align-items-center" />
                                    </MDBCol>
                                    <MDBCol>
                                        <span class="drone-name">{{ d.robot.display_name ?? d.robot.nid }}</span>
                                    </MDBCol>
                                    <MDBCol class="col-auto pe-3">
                                        <div class="status-dot"
                                            :style="{ backgroundColor: getStatusColor(d.robot.nid) }"></div>
                                    </MDBCol>
                                </MDBRow>
                            </MDBCol>
                        </MDBRow>
                    </transition>
                </MDBCol>

                <MDBCol class="col-auto menu mx-3 p-1 d-flex align-items-center clickable">
                    <button type="button" class="icon-button" @click="toggleMapFocus">
                        <OhVueIcon :name="isScrolledDown ? 'bi-chevron-double-up' : 'bi-chevron-double-down'"
                            scale="1.33" class="icon" />
                    </button>
                </MDBCol>

                <MDBCol class="col-auto menu mx-3 p-1 d-flex align-items-center clickable">
                    <button type="button" class="icon-button d-flex align-items-center gap-2"
                        @click="toggleOpenGeoTools">
                        <OhVueIcon name="ri-tools-line" scale="1.33" class="icon" />
                    </button>
                </MDBCol>

                <MDBCol class="col-auto menu mx-3 p-1 d-flex align-items-center clickable">
                    <button type="button" class="icon-button d-flex align-items-center gap-2" @click="toggleShow3dmesh">
                        <OhVueIcon :name="mapIconName" scale="1.33" class="icon" />
                    </button>
                </MDBCol>

                <MDBCol class="col-auto menu mx-3 p-1 d-flex align-items-center clickable">
                    <button type="button" class="icon-button d-flex align-items-center gap-2" @click="downloadData">
                        <OhVueIcon name="io-save-sharp" scale="1.33" class="icon" />
                    </button>
                </MDBCol>

                <MDBCol class="col-auto menu mx-3 p-1 d-flex align-items-center clickable">
                    <button type="button" class="icon-button d-flex align-items-center gap-2"
                        @click="triggerFilePicker">
                        <OhVueIcon name="md-drivefolderupload-round" scale="1.33" class="icon" />
                    </button>
                </MDBCol>

                <input type="file" ref="fileInput" accept=".json" style="display: none" @change="handleFileImport" />
            </MDBRow>
        </MDBCol>

        <MDBCol class="mx-3 d-flex align-items-start side-menu"
            :class="{ open: isAnyMenuOpen, closed: !isAnyMenuOpen }">
            <transition name="fade" mode="out-in">
                <component :is="isNodeMenuOpen ? NodeSideMenu : (isGeoToolsOpen ? GeoEditingMainMenu : null)"
                    key="side-content" />
            </transition>
        </MDBCol>
    </MDBRow>
</template>

<style scoped>
/* Utility */
.cursor-pointer {
    cursor: pointer;
}

.non-clickable {
    pointer-events: none;
}

.clickable {
    pointer-events: auto;
}

/* Main Layout */
.menu-root {
    position: fixed;
    top: 0;
    left: 0;
    width: calc(100vw - 40px);
    margin: 20px;
    z-index: 1000;
}

.menu {
    background: rgba(255, 255, 255, 0.65);
    border-radius: 12px;
    border: 1px solid rgba(255, 255, 255, 0.5);
    box-shadow: 0 3px 6px rgba(0, 0, 0, 0.15);
    backdrop-filter: blur(12px) saturate(150%);
    z-index: 1000;
    height: min-content;
}

/* Side Menu Panel */
.side-menu {
    transition: max-width 0.3s ease, opacity 0.3s ease, height 0.3s ease;
    overflow: hidden;
    opacity: 0;
    border-radius: 12px;
    height: calc(100vh - 140px);
}

.side-menu.open {
    max-width: 33vw;
    opacity: 1;
}

.side-menu.closed {
    max-width: 0;
    padding: 0 !important;
    margin: 0 !important;
    opacity: 0;
}

/* Inputs & Search */
.text-input-layer {
    background-color: rgba(255, 255, 255, 0.55);
    border-radius: 8px 0 0 8px;
}

.search-button-fix {
    border-radius: 0 8px 8px 0 !important;
}

.search-outline-fix {
    border: 1px rgba(92, 92, 92, 0.555) solid;
    border-radius: 8px 0 0 8px;
}

/* Drone List Entries */
.input-layer {
    background-color: rgba(255, 255, 255, 0.350);
    border-radius: 8px;
    border: 1px rgba(126, 126, 126, 0.178) solid;
}

.drone-entry {
    color: rgba(41, 41, 41, 0.955);
    transition: all 0.25s ease;
}

.drone-entry:hover {
    border-color: #399cff;
    background-color: rgba(25, 118, 210, 0.06);
}

/* Icons & Buttons */
.icon {
    width: fit-content;
    height: fit-content;
}

.icon-button {
    color: rgba(41, 41, 41, 0.955);
    background: transparent;
    border: none;
    cursor: pointer;
    padding: 0.3em;
}

.icon-button:hover .icon {
    color: #1976d2;
    transition: color 0.3s ease;
}

/* Status Dots & Animations */
.status-dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    border: 1px solid rgba(0, 0, 0, 0.2);
    transition: background-color 0.3s ease;
}

.status-dot[style*="background-color: #008000"] {
    box-shadow: 0 0 8px #0080007a;
}

.status-dot[style*="background-color: #ffff00"] {
    box-shadow: 0 0 4px rgba(255, 255, 0, 0.4);
}

.status-dot[style*="background-color: #ff0000"] {
    animation: status-pulse 1.5s infinite ease-in-out;
}

@keyframes status-pulse {

    0%,
    100% {
        transform: scale(1);
        box-shadow: 0 0 0px rgba(255, 0, 0, 0.7);
    }

    50% {
        transform: scale(1.2);
        box-shadow: 0 0 10px rgba(255, 0, 0, 0.9);
    }
}

/* Transitions */
.expand-fade-enter-active,
.expand-fade-leave-active {
    transition: all 0.25s ease;
}

.expand-fade-enter-from,
.expand-fade-leave-to {
    max-height: 0;
    opacity: 0;
    overflow: hidden;
}

.expand-fade-enter-to,
.expand-fade-leave-from {
    max-height: calc(100vh - 80px);
    opacity: 1;
}

.fade-enter-active,
.fade-leave-active {
    transition: opacity 0.25s ease;
}

.fade-enter-from,
.fade-leave-to {
    opacity: 0;
}

.title,
.title_small {
    font-family: Verdana, sans-serif;
    font-weight: bold;
    background: #000;
    background-clip: text;
    -webkit-text-fill-color: transparent;
    text-shadow: 0 1px 2px rgba(255, 255, 255, 0.3);
}

.title {
    font-size: xx-large;
}

.title_small {
    font-size: larger;
}
</style>