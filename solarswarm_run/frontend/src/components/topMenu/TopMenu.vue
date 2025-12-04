<script setup lang="ts">
import { computed, ref, watch, onMounted, onUnmounted, toRaw } from 'vue'
import { storeToRefs } from 'pinia'
import {
    Viewer,
    ScreenSpaceEventHandler,
    Cartesian2,
    defined,
    ScreenSpaceEventType,
    Entity as CesiumEntity,
    Entity
} from 'cesium'

import {
    MDBRow,
    MDBCol,
    MDBDropdown,
    MDBDropdownToggle,
    MDBDropdownMenu,
    MDBDropdownItem
} from 'mdb-vue-ui-kit'
import { OhVueIcon } from 'oh-vue-icons'

import NodeSideMenu from '../NodeInfo/NodeSideMenu.vue'
import GeoEditingMainMenu from '../GeoEditing/GeoEditingMainMenu.vue'

import { useDroneEntityStore } from '../../DronesData/DroneEntityStore'
import { useGeoToolsStore } from '../../stores/GeoToolsStore'
import { UseViewedDroneStore } from '../../stores/viewedDroneStore'
import type { Robot, RobotWithEntity } from '../../models/Robot'
import { useDroneDataStore } from '../../DronesData/DroneDataStore'
import { useSettingsStore } from '../../stores/SettingsStore'
import { useDroneHistoryStore } from '../../DronesData/DroneHistoryStore'
import { useTimeStore } from '../../stores/TimeStore'

// === Stores & refs ===
const viewedDroneStore = UseViewedDroneStore()
const { viewedRobot } = storeToRefs(viewedDroneStore)

const droneDataStore = useDroneDataStore()

const droneEntityStore = useDroneEntityStore()
const { viewer } = storeToRefs(droneEntityStore) // viewer is Ref<Viewer | null>

const geoStore = useGeoToolsStore()
const { isOpen: isGeoToolsOpen } = storeToRefs(geoStore)

// Local UI state
const showDroneList = ref(false)
// State to track if the user is focused on the top or bottom
const isScrolledDown = ref(false)

// Derived state — true if *any* menu should be open
const isAnyMenuOpen = computed(() => viewedRobot?.value || isGeoToolsOpen.value)
const isNodeMenuOpen = computed(() => viewedRobot?.value)

// Computed list of all drones with IDs and names
const allDrones = computed(() => {
    // Matches the shape returned by your entity store: { robot, entity }
    return droneDataStore.all() || []
})

// Add after your other refs
const droneHistoryStore = useDroneHistoryStore()

// Auto-refresh dronesAmount every 5s
let countInterval: number | undefined
onMounted(() => {
    countInterval = window.setInterval(() => {
        droneHistoryStore.activeRobotCount // Triggers cache refresh + recompute
    }, 5000)
})

onUnmounted(() => {
    if (countInterval) {
        window.clearInterval(countInterval)
    }
})

// Update computed to force cache refresh
const dronesAmount = computed(() => {
    return droneHistoryStore.activeRobotCount
})


const settingsStore = useSettingsStore();

// Close all menus
const closeAllMenus = () => {
    geoStore.setOpen(false)
    viewedDroneStore.setDrone(null)
    showDroneList.value = false
}

const mapIconName = computed(() => {
    return settingsStore.getShow3dMesh() ? "ri-map-2-fill" : "ri-map-2-line"
})
const toggleShow3dmesh = () => {
    settingsStore.setShow3dMesh(!settingsStore.getShow3dMesh())
}

// Open geo tools, close drone info
const toggleOpenGeoTools = () => {
    geoStore.toggleOpen()
    if (geoStore.isOpen) {
        viewedDroneStore.setDrone(null)
        useDroneEntityStore().removeFlightPathLines(viewedDroneStore.viewedNid ?? "")
    }
    showDroneList.value = false
}

/**
 * Focus / fly to a drone by nid
 */
const focusDrone = (nid: string) => {
    showDroneList.value = false

    const v = viewer.value
    if (!v) return
    const droneWithEntity = droneEntityStore.getRobot(nid) as RobotWithEntity | undefined
    if (!droneWithEntity || !droneWithEntity.entity) return

    useDroneEntityStore().removeFlightPathLines(viewedDroneStore.viewedNid ?? "")
    viewedDroneStore.setDrone(null, nid)
    geoStore.setOpen(false)

    droneEntityStore.drawFlightPath(nid, useTimeStore().currentTime, 500)

    v.flyTo(droneWithEntity.entity, { duration: 0.85 }).then(() => {
        v.trackedEntity = toRaw(droneWithEntity.entity)
    }).catch((e) => {
        console.log("error")
    })
}

/**
 * Toggles the main viewport scroll between the top and bottom of the page.
 */
const toggleMapFocus = () => {
    // Use document.documentElement for standard compatibility, fall back to document.body
    const targetElement = document.documentElement || document.body

    // Check current scroll position
    const currentScroll = targetElement.scrollTop || document.body.scrollTop
    const maxScroll = targetElement.scrollHeight - targetElement.clientHeight

    // Determine target position: 0 (top) or maxScroll (bottom)
    // We toggle based on whether the user is currently at the top or bottom (with a small tolerance)
    let targetScroll = 0

    // If we are close to the top, scroll to the bottom (maxScroll)
    if (currentScroll < 10) {
        targetScroll = maxScroll
        isScrolledDown.value = true
    } else {
        // Otherwise, scroll back to the top (0)
        targetScroll = 0
        isScrolledDown.value = false
    }

    // Scroll smoothly
    window.scrollTo({
        top: targetScroll,
        behavior: 'smooth'
    })
}

/* --------------------------
    Screen-space selection
    -------------------------- */

let handler: ScreenSpaceEventHandler | null = null

/**
 * Create the ScreenSpaceEventHandler on a viewer and wire double-click to focusDrone.
 * Ensures we clean up previous handler if present.
 */
function attachDroneSelector(v: Viewer) {
    // remove previous handler if any
    detachDroneSelector()

    // create only when scene.canvas exists
    if (!v.scene || !v.scene.canvas) return

    handler = new ScreenSpaceEventHandler(v.scene.canvas)

    handler.setInputAction((click: { position: Cartesian2 }) => {
        try {
            const picked = v.scene.pick(click.position)

            // picked might be undefined, a Primitive, an Entity, or a geometry object
            if (!defined(picked)) return

            const pickedId = (picked as any).id ?? (picked as any).primitive?.id

            if (!pickedId) return

            // If pickedId is a Cesium Entity (has properties), check isSelectable
            // also guard if properties.isSelectable is a ConstantProperty or plain boolean
            const isSelectableProp = pickedId.properties?.isSelectable
            let selectable = false
            if (typeof isSelectableProp === 'object' && isSelectableProp?.getValue) {
                selectable = !!isSelectableProp.getValue()
            } else {
                selectable = !!isSelectableProp
            }

            if (!selectable) return

            // droneData may be a ConstantProperty or object; attempt to read nid safely
            const droneDataProp = pickedId.properties?.droneData
            let nid: string | undefined = undefined

            if (droneDataProp && typeof droneDataProp.getValue === 'function') {
                const value = droneDataProp.getValue()
                nid = value?.nid
            } else if (pickedId.properties?.droneData) {
                nid = pickedId.properties.droneData?.nid
            }

            if (nid) focusDrone(nid)
        } catch (e) {
            // swallow pick errors to avoid noisy exceptions from Cesium internals
            // console.debug('pick error', e)
        }
    }, ScreenSpaceEventType.LEFT_DOUBLE_CLICK)
}

function detachDroneSelector() {
    if (handler) {
        try {
            handler.destroy()
        } catch {
            // ignore
        }
        handler = null
    }
}

// watch viewer ref and attach/detach handler
watch(
    viewer,
    (newViewer, oldViewer) => {
        if (newViewer) {
            attachDroneSelector(newViewer)
        } else {
            detachDroneSelector()
        }
    },
    { immediate: true }
)

// cleanup on unmount
onUnmounted(() => {
    detachDroneSelector()
    // ensure tracked entity cleared? optional:
    // if (viewer.value) viewer.value.trackedEntity = undefined
})

</script>

<template>
    <MDBRow class="menu-root non-clickable">
        <MDBCol>
            <MDBRow>
                <MDBCol class="col-auto menu mx-2 d-flex align-items-center clickable">
                    <div class="title">Solar Swarm</div>
                </MDBCol>

                <MDBCol class="col-auto menu mx-3 d-flex align-items-center clickable">
                    <div class="input-group">
                        <div class="form-outline search-outline-fix">
                            <input type="text" id="searchForm" class="form-control text-input-layer" />
                        </div>
                        <button type="button" class="btn btn-primary search-button-fix">
                            <OhVueIcon name="bi-search" scale="1" class="icon" />
                        </button>
                    </div>
                </MDBCol>

                <MDBCol class="col-auto menu mx-3 d-flex align-items-center clickable robot-dropdown">
                    <MDBDropdown v-model="showDroneList" popperClass="non-clickable noRipple">
                        <MDBDropdownToggle tag="div" class="d-flex align-items-center p-0"
                            @click="showDroneList = !showDroneList">
                            <div class="p-2 title_small">{{ dronesAmount }}</div>
                            <OhVueIcon name="la-robot-solid" scale="1.5" class="icon p-1" />
                        </MDBDropdownToggle>

                        <MDBDropdownMenu :class="{ 'd-block': showDroneList }" class="dropdown-menu-list">
                            <MDBDropdownItem v-for="drone in allDrones" :key="drone.robot.nid">
                                <p class="cursor-pointer" @click="focusDrone(drone.robot.nid)">
                                    🤖 {{ drone.robot.display_name || `Drone ${drone.robot.nid}` }}
                                </p>
                            </MDBDropdownItem>

                            <MDBDropdownItem v-if="dronesAmount === 0" disabled>
                                No active drones
                            </MDBDropdownItem>
                        </MDBDropdownMenu>
                    </MDBDropdown>
                </MDBCol>


                <MDBCol class="col-auto menu mx-3 d-flex align-items-center clickable">
                    <button type="button" class="icon-button" @click="toggleMapFocus">
                        <OhVueIcon :name="isScrolledDown ? 'bi-chevron-double-up' : 'bi-chevron-double-down'"
                            scale="1.5" class="icon" />
                    </button>
                </MDBCol>

                <MDBCol class="col-auto menu mx-3 d-flex align-items-center clickable">
                    <button type="button" class="icon-button d-flex align-items-center gap-2"
                        @click="toggleOpenGeoTools">
                        <OhVueIcon name="ri-tools-line" scale="1.33" class="icon" />
                    </button>
                </MDBCol>

                <MDBCol class="col-auto menu mx-3 d-flex align-items-center clickable">
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
    height: fit-content;
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
    background: rgba(255, 255, 255, 0.54);
    border-radius: 12px;
    border: 1px solid rgba(255, 255, 255, 0.5);
    box-shadow: 0 3px 6px rgba(0, 0, 0, 0.15);
    backdrop-filter: blur(8px) saturate(150%);
    -webkit-backdrop-filter: blur(8px) saturate(150%);
    z-index: 1000;
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
</style>
