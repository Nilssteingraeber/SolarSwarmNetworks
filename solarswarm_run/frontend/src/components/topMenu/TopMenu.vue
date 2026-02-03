<script setup lang="ts">
import { computed, ref, watch, onMounted, onUnmounted, toRaw } from 'vue'
import { storeToRefs } from 'pinia'
import {
    Viewer,
    ScreenSpaceEventHandler,
    Cartesian2,
    defined,
    ScreenSpaceEventType,
    HeadingPitchRange
} from 'cesium'

import {
    MDBRow,
    MDBCol
} from 'mdb-vue-ui-kit'
import { OhVueIcon } from 'oh-vue-icons'

import NodeSideMenu from '../NodeInfo/NodeSideMenu.vue'
import GeoEditingMainMenu from '../GeoEditing/GeoEditingMainMenu.vue'

import { useDroneEntityStore } from '../../DronesData/DroneEntityStore'
import { useGeoToolsStore } from '../../stores/GeoToolsStore'
import { UseViewedDroneStore } from '../../stores/viewedDroneStore'
import type { RobotWithEntity } from '../../models/Robot'
import { useSettingsStore } from '../../stores/SettingsStore'
import { useDroneHistoryStore } from '../../DronesData/DroneHistoryStore'
import { useTimeStore } from '../../stores/TimeStore'
import { degToRad } from 'three/src/math/MathUtils'

// === Stores ===
const viewedDroneStore = UseViewedDroneStore()
const { viewedRobot } = storeToRefs(viewedDroneStore)

const droneEntityStore = useDroneEntityStore()
const { viewer } = storeToRefs(droneEntityStore)

const geoStore = useGeoToolsStore()
const { isOpen: isGeoToolsOpen } = storeToRefs(geoStore)

const droneHistoryStore = useDroneHistoryStore()
const settingsStore = useSettingsStore()

// === UI state ===
const showDroneList = ref(false)
const isScrolledDown = ref(false)
const searchQuery = ref('')

// === Derived state ===
const isAnyMenuOpen = computed(() => viewedRobot?.value || isGeoToolsOpen.value)
const isNodeMenuOpen = computed(() => viewedRobot?.value)

const allDrones = computed(() => droneEntityStore.getRobots ?? [])

const filteredDrones = computed(() => {
    const q = searchQuery.value.trim().toLowerCase()
    if (!q) return allDrones.value

    return allDrones.value.filter(d =>
        d.robot.display_name?.toLowerCase().includes(q) ||
        d.robot.nid.toLowerCase().includes(q)
    )
})

const dronesAmount = computed(() => droneHistoryStore.activeRobotCount)

const mapIconName = computed(() =>
    settingsStore.getShow3dMesh() ? 'ri-map-2-fill' : 'ri-map-2-line'
)

// === Actions ===
const toggleShow3dmesh = () =>
    settingsStore.setShow3dMesh(!settingsStore.getShow3dMesh())

const toggleOpenGeoTools = () => {
    geoStore.toggleOpen()
    if (geoStore.isOpen) {
        viewedDroneStore.setDrone(null)
        droneEntityStore.removeFlightPathLines(viewedDroneStore.viewedNid ?? '')
    }
    showDroneList.value = false
}

const focusDrone = (nid: string) => {
    showDroneList.value = false
    searchQuery.value = ''

    const v = viewer.value
    if (!v) return

    const drone = droneEntityStore.getRobot(nid) as RobotWithEntity | undefined
    if (!drone?.entity) return

    droneEntityStore.removeFlightPathLines(viewedDroneStore.viewedNid ?? '')
    viewedDroneStore.setDrone(null, nid)
    geoStore.setOpen(false)

    droneEntityStore.drawFlightPath(nid, useTimeStore().currentTime, 500)

    // --- Fly to drone with controlled zoom + north-facing orientation ---
    v.flyTo(drone.entity, {
        duration: 0.35,
        offset: new HeadingPitchRange(
            0,                              // heading = north
            degToRad(-35),     // look down
            120                             // distance in meters (tweak this)
        )
    }).then(() => {
        // Enable tracking
        v.trackedEntity = toRaw(drone.entity)

        // --- Force camera back to north (tracking overrides orientation) ---
        const pos = drone?.entity?.position?.getValue(v.clock.currentTime)
        if (!pos) return

        v.camera.setView({
            destination: v.camera.position,
            orientation: {
                heading: 0,                               // north
                pitch: v.camera.pitch,
                roll: 0
            }
        })
    })
}


const toggleMapFocus = () => {
    const target = document.documentElement
    const current = target.scrollTop
    const max = target.scrollHeight - target.clientHeight
    isScrolledDown.value = current < 10
    window.scrollTo({ top: isScrolledDown.value ? max : 0, behavior: 'smooth' })
}

// === Cesium selection ===
let handler: ScreenSpaceEventHandler | null = null

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
                    <MDBRow class="my-2 w-100">
                        <div class="input-group px-0">
                            <div class="form-outline search-outline-fix">
                                <input type="text" id="searchForm" class="form-control text-input-layer h-100"
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
                        </div>
                    </MDBRow>

                    <transition name="expand-fade">
                        <MDBRow class="w-100 px-0" v-if="showDroneList">
                            <MDBCol class="drone-list px-2">
                                <MDBRow v-for="d in [...filteredDrones]" :key="d.robot.nid"
                                    @click="focusDrone(d.robot.nid)"
                                    class="p-1 px-0 cursor-pointer drone-entry d-flex align-items-center input-layer my-2">
                                    <MDBCol class="col-auto px-0">
                                        <OhVueIcon name="la-robot-solid" scale="1"
                                            class="icon p-1 ps-2 d-flex align-items-center" />
                                    </MDBCol>

                                    <MDBCol>
                                        <span class="drone-name">
                                            {{ d.robot.display_name ?? d.robot.nid }}
                                        </span>
                                    </MDBCol>
                                </MDBRow>

                            </MDBCol>
                        </MDBRow>
                    </transition>
                </MDBCol>

                <MDBCol class="col-auto menu mx-3 p-1 d-flex align-items-center clickable">
                    <button type="button" class="icon-button" @click="toggleMapFocus">
                        <OhVueIcon :name="isScrolledDown ? 'bi-chevron-double-up' : 'bi-chevron-double-down'"
                            scale="1.5" class="icon" />
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
.cursor-pointer {
    cursor: pointer;
}

.dropdown-menu-list {
    position: absolute;
    top: 100%;
    left: 0;
    margin-top: 25px;
    background: rgba(255, 255, 255, 0.9);
    border-radius: 8px;
    border: 1px solid rgba(255, 255, 255, 0.7);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
    backdrop-filter: blur(5px);
    z-index: 1001;
    min-width: 200px;
    padding: 8px 0;
}

/* Ensure the MDBDropdownToggle doesn't interfere with the list positioning */
.MDBDropdownToggle {
    height: 100%;
    align-items: center;
}

/* Animation */
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

/* Optional fade between menus */
.fade-enter-active,
.fade-leave-active {
    transition: opacity 0.25s ease;
}

.fade-enter-from,
.fade-leave-to {
    opacity: 0;
}

/* Click behavior */
.non-clickable {
    pointer-events: none;
}

.clickable {
    pointer-events: auto;
}

/* Input styles */
.text-input-layer {
    background-color: rgba(255, 255, 255, 0.55);
    border-top-left-radius: 8px;
    border-bottom-left-radius: 8px;
    border-top-right-radius: 0;
    border-bottom-right-radius: 0;
}

.search-button-fix {
    margin-right: -2px;
    border-top-right-radius: 8px !important;
    border-bottom-right-radius: 8px !important;
}

.search-outline-fix {
    border: 1px rgba(92, 92, 92, 0.555) solid;
    margin-left: -5px;
    border-radius: 8px 0 0 8px;
}

/* Icons */
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

/* Titles */
.title_small {
    font-family: Verdana, Geneva, Tahoma, sans-serif;
    font-size: larger;
    font-weight: bold;
    background: rgba(0, 0, 0, 1);
    background-clip: text;
    -webkit-text-fill-color: transparent;
    text-shadow: 0 1px 2px rgba(255, 255, 255, 0.3), 0 0 8px rgba(255, 255, 255, 0.5);
}

.title {
    font-family: Verdana, Geneva, Tahoma, sans-serif;
    font-size: xx-large;
    background: rgba(0, 0, 0, 1);
    background-clip: text;
    -webkit-text-fill-color: transparent;
    text-shadow: 0 1px 2px rgba(255, 255, 255, 0.3), 0 0 8px rgba(255, 255, 255, 0.5);
}

/* Layout */
.menu-root {
    position: fixed;
    top: 0;
    left: 0;
    width: calc(100vw - 40px);
    margin: 20px;
    height: auto;
    z-index: 1000;
}

.menu {
    background: rgba(255, 255, 255, 0.65);
    border-radius: 12px;
    border: 1px solid rgba(255, 255, 255, 0.5);
    box-shadow: 0 3px 6px rgba(0, 0, 0, 0.15);
    backdrop-filter: blur(12px) saturate(150%);
    -webkit-backdrop-filter: blur(12px) saturate(150%);
    z-index: 1000;
    height: min-content;
}

/* Anchor the dropdown to the robot-count button */
.robot-dropdown {
    position: relative;
}

/* Clean dropdown positioning */
.dropdown-menu-list {
    position: absolute;
    top: 100%;
    /* directly below the toggle */
    left: 0;
    margin-top: 6px;
    /* tiny gap, remove flicker */

    background: rgba(255, 255, 255, 0.9);
    border-radius: 8px;
    border: 1px solid rgba(255, 255, 255, 0.7);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
    backdrop-filter: blur(5px);

    z-index: 2000;
    /* ensures no flicker due to pointer-events */
    min-width: 200px;
    padding: 8px 0;
}

/* Fix MDB toggle height calculation */
.MDBDropdownToggle {
    display: flex;
    align-items: center !important;
}

.cursor-pointer {
    cursor: pointer;
}

.hover\:bg-gray-100:hover {
    background-color: #f2f2f2;
}

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

.input-layer {
    background-color: rgba(255, 255, 255, 0.350);
    border-radius: 8px;
    border: 1px rgba(126, 126, 126, 0.178) solid;
}

/* Base color for text + icon */
.drone-entry {
    color: rgba(41, 41, 41, 0.955);
    transition: color 0.25s ease, background-color 0.25s ease;
}

/* Make icons inherit text color */
.drone-entry .icon {
    color: inherit;
}

/* Hover state = EVERYTHING turns blue */
.drone-entry:hover {
    border-color: #399cff;
    background-color: rgba(25, 118, 210, 0.06);
}

/* Optional: subtle hover polish */
.drone-entry:hover .drone-name {
    text-decoration: none;
}
</style>
