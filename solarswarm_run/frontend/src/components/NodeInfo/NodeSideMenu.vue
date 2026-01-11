<script setup lang="ts">
import { MDBRow, MDBCol } from 'mdb-vue-ui-kit'
import NodePowerOffButton from './NodePowerOffButton.vue';
import { OhVueIcon } from 'oh-vue-icons';
import Node3DTransform from './Node3DTransform.vue';
import NodeCPUGraph from './NodeCPUGraph.vue';
import { computed, toRaw } from 'vue';
import ServicesGraph from './Services/ServicesGraph.vue';
import Toggle from '../controlSurfaces/Toggle.vue';
import { UseViewedDroneStore } from '../../stores/viewedDroneStore';
import { storeToRefs } from 'pinia';
import { useDroneEntityStore } from '../../DronesData/DroneEntityStore';
import { useTimeStore } from '../../stores/TimeStore';


const droneStore = useDroneEntityStore()
const viewedDroneStore = UseViewedDroneStore()
const { currentTime } = useTimeStore()
const { viewer } = storeToRefs(droneStore)
const { viewedRobot } = storeToRefs(viewedDroneStore)


const batteryIconNames = ["fa-battery-empty", "fa-battery-quarter", "fa-battery-half", "fa-battery-three-quarters", "fa-battery-full"]
const batteryIconColors = ["", "red", "yellow", "green", "green"]

const wifiIconNames = ["bi-wifi", "bi-wifi-2", "bi-wifi-1", "bi-wifi-off"]
const wifiIconColors = ["green", "yellow", "orange", "red"]



const wifiLevelStatus = computed(() => {

    // 1. Time-Since-Last Check (Critical Disconnect)
    const lastHeard = viewedRobot?.value?.last_heard || 0
    const timeSinceLast = (currentTime - lastHeard) / 1000
    if (timeSinceLast >= 7) return 3 // Status 3: Disconnected / Timed out

    // 2. Determine Overall Connectivity (Highest Strength)
    const connectivityMap = viewedRobot?.value?.connectivity;
    const connectivityValues = connectivityMap ? Object.values(connectivityMap) : [];

    let highestStrength = 0;
    if (connectivityValues.length > 0) {
        highestStrength = Math.max(...connectivityValues);
    } else {
        // If the drone is active but has no recorded neighbors
        return 3;
    }

    // 3. Map Highest Strength to Status (0: Best, 3: Worst)
    // The scale remains the same for simplicity:
    // > 0.666 (Strong) => 0
    // > 0.333 (Medium) => 1
    // > 0.0   (Weak)   => 2
    // <= 0.0 (None)   => 3

    if (highestStrength > 0.666) return 0;
    if (highestStrength > 0.333) return 1;
    if (highestStrength > 0.0) return 2;

    // Fallback if highestStrength is 0.0
    return 3;
})

const currentTimeString = computed(() => {
    if (!viewedRobot?.value?.last_heard) return "Never"

    const date = new Date(viewedRobot?.value?.last_heard);
    const hours = String(date.getHours()).padStart(2, '0')
    const minutes = String(date.getMinutes()).padStart(2, '0')
    const seconds = String(date.getSeconds()).padStart(2, '0')
    return `${hours}:${minutes}:${seconds}`
})


const isOpen = computed(() => {
    //console.log("Robot: ", viewedRobot?.value, viewedRobot?.value !== undefined)
    return true
})
const nickname = computed(() => {
    return viewedRobot?.value?.display_name ?? viewedRobot?.value?.nid ?? "---"
})


const batteryLevel = 0.78
const currentBatteryStatus = computed(() => {
    const val = (viewedRobot?.value?.battery ?? 0) / 100.0
    return (val > 0.75 ? 4 : val > 0.5 ? 3 : val > 0.25 ? 2 : val > 0 ? 1 : 0)

});


const batteryLevelText = computed(() => {
    return Math.round(viewedRobot?.value?.battery ?? 0).toString()
})


const closeMenu = () => {
    useDroneEntityStore().removeFlightPathLines(viewedDroneStore.viewedNid ?? "")
    viewedDroneStore.setDrone(null);
    if (viewer.value)
        viewer.value.trackedEntity = undefined;
}

</script>

<template>
    <MDBRow class="menu side-menu-root justify-content-end" :class="{ 'menu-closed': !isOpen }">
        <MDBCol class="col-auto text-center h-100 d-flex flex-column">
            <!-- Header row (fixed at top of menu) -->
            <MDBRow class="flex justify-content-between align-items-center header-row">
                <MDBCol class="col-auto title">{{ nickname }}</MDBCol>
                <MDBCol />
                <MDBCol class="col-auto p-2">
                    <OhVueIcon name="io-close" scale="1.5" class="input-layer close-button" @click="closeMenu" />
                </MDBCol>
            </MDBRow>

            <!-- Scrollable content -->
            <MDBRow class="menu-content flex-grow-1 overflow-auto custom-scroll">
                <MDBCol>
                    <!-- Battery -->
                    <MDBRow class="p-2">
                        <MDBCol class="col-auto">
                            <MDBRow class="justify-content-center">
                                <MDBCol class="col-12">
                                    <span class="status-bar-info-text">Battery</span>
                                </MDBCol>
                            </MDBRow>
                            <MDBRow class="justify-content-center">
                                <MDBCol class="col-auto battery-center">
                                    <OhVueIcon :name="batteryIconNames[currentBatteryStatus]" scale="2.3"
                                        :class="batteryIconColors[currentBatteryStatus]"></OhVueIcon>
                                    <div class="battery-level-text">
                                        {{ batteryLevelText }}
                                    </div>
                                </MDBCol>
                            </MDBRow>
                        </MDBCol>

                        <!-- Connection -->
                        <MDBCol class="col-auto">
                            <MDBRow class="justify-content-center">
                                <MDBCol class="col-12">
                                    <span class="status-bar-info-text">Connection</span>
                                </MDBCol>
                            </MDBRow>
                            <MDBRow class="justify-content-center">
                                <MDBCol class="col-auto d-flex align-items-center">
                                    <OhVueIcon :name="wifiIconNames[wifiLevelStatus]" scale="1.9"
                                        :class="'wifi-front ' + wifiIconColors[wifiLevelStatus]">
                                        <OhVueIcon v-if="wifiLevelStatus != 3" :name="wifiIconNames[0]" scale="1.9"
                                            class="wifi-backdrop">
                                        </OhVueIcon>
                                    </OhVueIcon>

                                </MDBCol>
                            </MDBRow>
                        </MDBCol>
                        <MDBCol />

                        <!-- Restart Button -->
                        <!-- <MDBCol class="col-auto">
                    <MDBRow class="justify-content-center">
                        <MDBCol class="col-12">
                            <span class="status-bar-info-text">Restart</span>
                        </MDBCol>
                    </MDBRow>
                    <MDBRow class="justify-content-center">
                        <MDBCol class="input-layer m-2 col-auto d-flex align-items-center p-2">
                            <NodeRestartButton />
                        </MDBCol>
                    </MDBRow>
                </MDBCol> -->

                        <!-- Power Button -->

                        <MDBCol class="col-auto">
                            <MDBRow class="justify-content-center mb-0 pb-0">
                                <MDBCol class="col-12">
                                    <span class="status-bar-info-text">Shut-Down</span>
                                </MDBCol>
                            </MDBRow>
                            <MDBRow class="justify-content-center">
                                <NodePowerOffButton />
                            </MDBRow>
                        </MDBCol>

                    </MDBRow>

                    <MDBRow class="justify-content-end">
                        <MDBCol class="col-auto flex-column align-items-start">
                            <div>
                                <span class="status-bar-text-banner-small"> {{ currentTimeString }}</span>
                                <span class="status-bar-text-banner"> {{ " " }} </span>
                                <span class="status-bar-text-banner"> {{ " 24.02.2025 " }} </span>
                            </div>
                        </MDBCol>

                    </MDBRow>

                    <MDBRow class="justify-content-center mb-2">
                        <MDBCol>
                            <hr class="divider">
                        </MDBCol>
                    </MDBRow>

                    <MDBRow class="justify-content-between p-2">
                        <MDBCol class="status-bar-text-banner-small">
                            <MDBRow class="mb-1 align-items-stretch">
                                <MDBCol class="py-2 px-3 mx-2 mb-2 h-100">
                                    <!-- Toggle -->
                                    <MDBRow class="mb-3">
                                        <MDBCol>
                                            <Toggle label="Pos." :toggle-items="['Lat Lon', 'XYZ']" />
                                        </MDBCol>
                                    </MDBRow>

                                    <!-- Lat -->
                                    <MDBRow>
                                        <MDBCol class="d-flex flex-nowrap justify-content-between">
                                            <p class="mb-0 math-text">Lat</p>
                                            <p class="mb-0 font-mono">{{ (viewedRobot?.point?.lat ?? 0).toFixed(7) }}
                                            </p>
                                        </MDBCol>
                                    </MDBRow>

                                    <!-- Lon -->
                                    <MDBRow class="mb-1">
                                        <MDBCol class="d-flex flex-nowrap justify-content-between">
                                            <p class="mb-0 math-text">Lon</p>
                                            <p class="mb-0 font-mono">{{ (viewedRobot?.point?.lon ?? 0).toFixed(7) }}
                                            </p>
                                        </MDBCol>
                                    </MDBRow>

                                    <!-- Alt -->
                                    <MDBRow>
                                        <MDBCol class="d-flex flex-nowrap justify-content-between">
                                            <p class="mb-0 math-text">Alt</p>
                                            <p class="mb-0 font-mono">{{ (viewedRobot?.point?.alt ?? 0).toFixed(2) +
                                                " m" }}
                                            </p>
                                        </MDBCol>
                                    </MDBRow>

                                </MDBCol>

                                <MDBCol class="py-2 px-3 mx-2 h-100">
                                    <!-- Toggle -->
                                    <MDBRow class="mb-2">
                                        <MDBCol>
                                            <Toggle label="Rot." :toggle-items="['World', 'Local']" />
                                        </MDBCol>
                                    </MDBRow>
                                    <!-- Rotation -->
                                    <MDBRow class="mb-0">
                                        <MDBCol class="d-flex flex-nowrap justify-content-between">
                                            <p class="mb-0 math-text text-x">x</p>
                                            <p class="mb-0">1.000000</p>
                                        </MDBCol>
                                    </MDBRow>
                                    <MDBRow class="mb-0">
                                        <MDBCol class="d-flex flex-nowrap justify-content-between">
                                            <p class="mb-0 math-text text-y">y</p>
                                            <p class="mb-0">1.000000</p>
                                        </MDBCol>
                                    </MDBRow>
                                    <MDBRow class="mb-0">
                                        <MDBCol class="d-flex flex-nowrap justify-content-between">
                                            <p class="mb-0 math-text text-z">z</p>
                                            <p class="mb-0">1.000000</p>
                                        </MDBCol>
                                    </MDBRow>
                                    <MDBRow class="mb-0">
                                        <MDBCol class="d-flex flex-nowrap justify-content-between">
                                            <p class="mb-0 math-text text-w">w</p>
                                            <p class="mb-0">1.000000</p>
                                        </MDBCol>
                                    </MDBRow>
                                </MDBCol>
                                <MDBCol class="m-2 h-100">
                                    <Node3DTransform></Node3DTransform>
                                </MDBCol>
                            </MDBRow>
                        </MDBCol>
                    </MDBRow>

                    <MDBRow class="justify-content-start p-2 mb-5 min-graph-height ">
                        <MDBCol class="col-6 h-100">
                            <NodeCPUGraph title="CPU %" dataFieldName="cpu_1"></NodeCPUGraph>
                        </MDBCol>
                        <MDBCol class="col-6 h-100">
                            <NodeCPUGraph title="RAM %" dataFieldName="ram_1"></NodeCPUGraph>
                        </MDBCol>
                    </MDBRow>


                    <MDBRow class="justify-content-between p-2 h-100">
                        <MDBCol>
                            <ServicesGraph></ServicesGraph>
                        </MDBCol>
                    </MDBRow>

                </MDBCol>
            </MDBRow>
        </MDBCol>
    </MDBRow>

</template>

<style scoped>
.min-graph-height {
    height: 15ch;
}

@font-face {
    font-family: 'Latin Modern Math';
    src: url("@/assets/latinmodern-math.otf") format('opentype');
}

.text-x {
    color: rgb(212, 58, 58)
}

.text-y {
    color: rgb(46, 177, 64)
}

.text-z {
    color: rgb(58, 61, 212)
}

.math-text {
    font-family: 'Latin Modern Math', serif;
    font-size: 12px;
}

.font-mono {
    font-family: monospace;
}

.side-menu-root {
    pointer-events: none;
    z-index: 5000;
    height: calc(100vh - 65px);
    overflow-y: visible;
}

.side-menu-root * {
    pointer-events: auto;
}

.status-bar-text-banner-small {
    font-size: 12px;
    font-weight: bold;
}

.status-bar-text-banner {
    font-size: 11px;
    font-weight: bold;
}

.status-bar-info-text {
    font-size: 12px;
    font-weight: bold;
    margin: 0 0 0 0 !important;
    padding: 0 !important;
}

.focus-button {
    transition: all 0.3s ease;
}

.focus-button:hover {
    color: rgb(56, 107, 192);
    background-color: rgba(255, 255, 255, 1);
    cursor: pointer;
}

.wifi-front {
    z-index: 2;
}

.wifi-backdrop {
    opacity: 0.3;
    position: absolute;
    z-index: 1;
}

.battery-center {
    position: relative;
}

.battery-level-text {
    position: absolute;
    font-size: 14px;
    font-weight: bold;
    left: 36%;
    top: 25%;
    text-shadow: -1px -1px 0 #ffffff, 1px -1px 0 #ffffff, -1px 1px 0 #ffffff, 1px 1px 0 #ffffff;
}

.green {
    color: rgb(53, 182, 59);
}

.orange {
    color: rgb(235, 101, 24);
}

.yellow {
    color: rgb(235, 161, 24);
}

.red {
    color: rgb(212, 58, 58);
}

.divider {
    margin: 5px;
    padding: 5;
}

.background-white {
    background-color: rgba(255, 255, 255, 0.350);
    border-radius: 8px;
    border: 1px rgba(126, 126, 126, 0.178) solid;
}

.input-layer {
    background-color: rgba(255, 255, 255, 0.350);
    border-radius: 8px;
    border: 1px rgba(126, 126, 126, 0.178) solid;
}

.close-button {
    transition: all 0.3s ease;
}

.close-button:hover {
    color: red;
    background-color: rgba(255, 255, 255, 1);
    cursor: pointer;
}

.search-button-fix {
    margin-right: -2px;
}

.search-outline-fix {
    border-top-left-radius: 8px;
    border-bottom-left-radius: 8px;
    border-top-right-radius: 0px;
    border-bottom-right-radius: 0px;
    border: 1px rgba(92, 92, 92, 0.555) solid;
    margin-left: -5px;
}

.icon {
    width: fit-content;
    height: fit-content;
}

.icon-button {
    background: transparent;
    border: none;
    cursor: pointer;
    padding: 0.3em;
}

.icon-button:hover .icon {
    color: #1976d2;
    transition: color 0.3s ease;
}

.shadowy {
    background: rgba(0, 0, 0, 1.0);
    background-clip: text;
    -webkit-text-fill-color: transparent;

    text-shadow: 0 1px 2px rgba(255, 255, 255, 0.3), 0 0 8px rgba(255, 255, 255, 0.5);
}

.title {
    font-family: Verdana, Geneva, Tahoma, sans-serif;
    font-size: xx-large;

    background: rgba(0, 0, 0, 1.0);
    background-clip: text;
    -webkit-text-fill-color: transparent;

    text-shadow: 0 1px 2px rgba(255, 255, 255, 0.3), 0 0 8px rgba(255, 255, 255, 0.5);
}

.sticky {
    position: sticky;
    /* makes it sticky */
    top: 0px;
    /* distance from the top of the scroll container */
    z-index: 2000;
    /* semi-transparent white */
    backdrop-filter: blur(1px)
}

.overflow {
    overflow: auto;
}

.menu {
    background: rgba(255, 255, 255, 0.65);
    border-radius: 12px;
    border: 1px solid rgba(255, 255, 255, 0.5);
    box-shadow: 0 3px 6px rgba(0, 0, 0, 0.15);
    backdrop-filter: blur(8px) saturate(300%);
    -webkit-backdrop-filter: blur(1px) saturate(150%);
    /* Fill the container */
    overflow-y: hidden;
    /* Enable internal scrolling if content exceeds height */
}

.custom-scroll {
    overflow-y: auto;
    /* enables vertical scrollbar */
    scrollbar-width: thin;
    /* Firefox: thin scrollbar */
    scrollbar-color: rgba(136, 136, 136, 0.7) transparent;
    /* thumb | track */
}

/* Chrome, Edge, Safari */
.custom-scroll::-webkit-scrollbar {
    width: 12px;
}

.custom-scroll::-webkit-scrollbar-track {
    background: transparent;
}

.custom-scroll::-webkit-scrollbar-thumb {
    background-color: rgba(136, 136, 136, 0.7);
    /* semi-transparent thumb */
    border-radius: 6px;
}


.side-menu-root {
    transition: right 0.4s ease-in-out;
    /* smooth slide */
}

.menu-closed {
    right: -100%;
    /* move completely offscreen */
}
</style>
