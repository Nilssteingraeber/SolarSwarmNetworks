<script setup lang="ts">
import { computed, ref, Ref, watch } from 'vue';
import { storeToRefs } from 'pinia';
import { MDBRow, MDBCol } from 'mdb-vue-ui-kit';
import { OhVueIcon } from 'oh-vue-icons';

import NodeCPUGraph from './NodeCPUGraph.vue';
import Toggle from '../ControlSurfaces/Toggle.vue';
import ServiceFactory from './Services/ServiceFactory.vue';
// Note: Node3DTransform, NodeRestartButton, NodePowerOffButton were used in template 
// but missing from imports in original snippet. Ensure they are registered globally or imported.

import { UseViewedDroneStore } from '../../stores/viewedDroneStore';
import { useDroneEntityStore } from '../../dronesData/DroneEntityStore';
import { useTimeStore } from '../../stores/TimeStore';
import { useServiceInterfaceStore } from '../../stores/ServiceInterfaceStore';
import { NamedParsedInterface } from '../../dronesData/DronesRosInterfaces';

// # Stores
const droneStore = useDroneEntityStore();
const { viewer } = storeToRefs(droneStore);

const viewedDroneStore = UseViewedDroneStore();
const { viewedRobot, viewedNid, dronesPollingService } = storeToRefs(viewedDroneStore);

const { currentTime } = useTimeStore();
const interfaceStore = useServiceInterfaceStore();

// # Config & Constants
const batteryIconNames = ["fa-battery-empty", "fa-battery-quarter", "fa-battery-half", "fa-battery-three-quarters", "fa-battery-full"];
const batteryIconColors = ["", "red", "yellow", "green", "green"];
const wifiIconNames = ["bi-wifi", "bi-wifi-2", "bi-wifi-1", "bi-wifi-off"];
const wifiIconColors = ["green", "yellow", "orange", "red"];

// # State
const servicesList = ref<Object | undefined>(undefined);
const servicesRef = ref<NamedParsedInterface[] | undefined>(undefined);
const servicesLoading = ref(false);

// # Computed Props
const isOpen = computed(() => true); // Placeholder for visibility logic

const nickname = computed(() => {
    return viewedRobot?.value?.display_name ?? viewedRobot?.value?.nid ?? "Unknown Name or NID";
});

const currentTimeString = computed(() => {
    if (!viewedRobot?.value?.last_heard) return "Never";
    return new Date(viewedRobot?.value.last_heard);
});

const wifiLevelStatus = computed(() => {
    // 1. Timeout Check
    const lastHeard = viewedRobot?.value?.last_heard || 0;
    if ((currentTime - lastHeard) / 1000 >= 7) return 3;

    const map = viewedRobot?.value?.connectivity;
    const values = map ? Object.values(map) as number[] : [];
    if (!values.length) return 3;

    const max = Math.max(...values);
    if (max > 0.666) return 0;
    if (max > 0.333) return 1;
    return max > 0.0 ? 2 : 3;
});

const currentBatteryStatus = computed(() => {
    const val = (viewedRobot?.value?.battery ?? 0) / 100.0;
    if (val > 0.75) return 4;
    if (val > 0.5) return 3;
    if (val > 0.25) return 2;
    return val > 0 ? 1 : 0;
});

const batteryLevelText = computed(() => {
    const batt = viewedRobot?.value?.battery;
    return (batt && batt.toString() === "-1") ? "?" : Math.round(batt ?? 0).toString();
});

// # Methods
const closeMenu = () => {
    useDroneEntityStore().removeFlightPathLines(viewedDroneStore.viewedNid ?? "");
    viewedDroneStore.setDrone(null);
    if (viewer.value) viewer.value.trackedEntity = undefined;
};

// # Watchers
watch(viewedNid, async (newNid) => {
    if (!newNid) {
        servicesRef.value = undefined;
        return;
    }

    // Check Cache
    if (interfaceStore.has(newNid).value) {
        servicesRef.value = interfaceStore.get(newNid).value;
        servicesLoading.value = false;
        return;
    }

    // Fetch
    servicesLoading.value = true;
    try {
        const data = await dronesPollingService?.value?.fetchServicesList(newNid);
        if (data) {
            interfaceStore.set(newNid, data);
            servicesRef.value = data;
        }
    } catch (err) {
        console.error('Failed to fetch services', err);
        servicesRef.value = undefined;
    } finally {
        servicesLoading.value = false;
    }
}, { immediate: true });
</script>

<template>
    <MDBRow class="menu side-menu-root justify-content-end h-100" :class="{ 'menu-closed': !isOpen }">
        <MDBCol class="col-auto text-center h-100 d-flex flex-column">

            <MDBRow class="flex justify-content-between align-items-center header-row">
                <MDBCol class="col-auto title">{{ nickname }}</MDBCol>
                <MDBCol />
                <MDBCol class="col-auto p-2">
                    <OhVueIcon name="io-close" scale="1.5" class="input-layer close-button" @click="closeMenu" />
                </MDBCol>
            </MDBRow>

            <MDBRow class="custom-scroll">
                <MDBCol>

                    <MDBRow class="p-2">
                        <MDBCol class="col-auto">
                            <MDBRow class="justify-content-center">
                                <MDBCol class="col-12"><span class="status-bar-info-text">Battery</span></MDBCol>
                            </MDBRow>
                            <MDBRow class="justify-content-center">
                                <MDBCol class="col-auto battery-center">
                                    <OhVueIcon :name="batteryIconNames[currentBatteryStatus]" scale="2.3"
                                        :class="batteryIconColors[currentBatteryStatus]" />
                                    <div class="battery-level-text">{{ batteryLevelText }}</div>
                                </MDBCol>
                            </MDBRow>
                        </MDBCol>

                        <MDBCol class="col-auto">
                            <MDBRow class="justify-content-center">
                                <MDBCol class="col-12"><span class="status-bar-info-text">Connection</span></MDBCol>
                            </MDBRow>
                            <MDBRow class="justify-content-center">
                                <MDBCol class="col-auto d-flex align-items-center">
                                    <OhVueIcon :name="wifiIconNames[wifiLevelStatus]" scale="1.9"
                                        :class="'wifi-front ' + wifiIconColors[wifiLevelStatus]">
                                        <OhVueIcon v-if="wifiLevelStatus != 3" :name="wifiIconNames[0]" scale="1.9"
                                            class="wifi-backdrop" />
                                    </OhVueIcon>
                                </MDBCol>
                            </MDBRow>
                        </MDBCol>

                        <MDBCol />

                        <MDBCol class="col-auto">
                            <MDBRow class="justify-content-center mb-0 pb-0">
                                <MDBCol class="col-12"><span class="status-bar-info-text">Shut-Down</span></MDBCol>
                            </MDBRow>
                            <MDBRow class="justify-content-center">
                            </MDBRow>
                        </MDBCol>
                    </MDBRow>

                    <MDBRow class="justify-content-end">
                        <MDBCol class="col-auto flex-column align-items-start">
                            <div>
                                <span class="status-bar-text-banner-small"> {{ currentTimeString }}</span>
                                <span class="status-bar-text-banner"> 24.02.2025 </span>
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
                                    <MDBRow class="mb-3">
                                        <MDBCol>
                                            <Toggle label="Pos." :toggle-items="['Lat Lon', 'XYZ']" />
                                        </MDBCol>
                                    </MDBRow>

                                    <MDBRow>
                                        <MDBCol class="d-flex flex-nowrap justify-content-between">
                                            <p class="mb-0 math-text">Lat</p>
                                            <p class="mb-0 font-mono">{{ (viewedRobot?.point?.lat ??
                                                viewedRobot?.point?.x ?? 0).toFixed(7) }}</p>
                                        </MDBCol>
                                    </MDBRow>

                                    <MDBRow class="mb-1">
                                        <MDBCol class="d-flex flex-nowrap justify-content-between">
                                            <p class="mb-0 math-text">Lon</p>
                                            <p class="mb-0 font-mono">{{ (viewedRobot?.point?.lon ??
                                                viewedRobot?.point?.y ?? 0).toFixed(7) }}</p>
                                        </MDBCol>
                                    </MDBRow>

                                    <MDBRow>
                                        <MDBCol class="d-flex flex-nowrap justify-content-between">
                                            <p class="mb-0 math-text">Alt</p>
                                            <p class="mb-0 font-mono">{{ (viewedRobot?.point?.alt ??
                                                viewedRobot?.point?.z ?? 0).toFixed(2) + " m" }}</p>
                                        </MDBCol>
                                    </MDBRow>
                                </MDBCol>

                                <MDBCol class="py-2 px-3 mx-2 h-100">
                                    <MDBRow class="mb-2">
                                        <MDBCol>
                                            <Toggle label="Rot." :toggle-items="['World', 'Local']" />
                                        </MDBCol>
                                    </MDBRow>
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

                    <MDBRow class="justify-content-center mb-2">
                        <MDBCol class="col-12">
                            <div class="m-2"></div>
                        </MDBCol>
                        <MDBCol class="col-12 sub_title">Services</MDBCol>
                    </MDBRow>
                    <MDBRow class="justify-content-between p-2 mb-3">
                        <MDBCol>
                            <ServiceFactory />
                        </MDBCol>
                    </MDBRow>

                </MDBCol>
            </MDBRow>
        </MDBCol>
    </MDBRow>
</template>

<style scoped>
/* # --- Fonts */
@font-face {
    font-family: 'Latin Modern Math';
    src: url("@/assets/latinmodern-math.otf") format('opentype');
}

.title {
    font-family: Verdana, sans-serif;
    font-size: xx-large;
    background: black;
    background-clip: text;
    -webkit-text-fill-color: transparent;
    text-shadow: 0 1px 2px rgba(255, 255, 255, 0.3), 0 0 8px rgba(255, 255, 255, 0.5);
}

.sub_title {
    font-family: Verdana, sans-serif;
    font-size: large;
    background: black;
    background-clip: text;
    -webkit-text-fill-color: transparent;
    text-shadow: 0 1px 2px rgba(255, 255, 255, 0.3), 0 0 8px rgba(255, 255, 255, 0.5);
}

.math-text {
    font-family: 'Latin Modern Math', serif;
    font-size: 12px;
}

.font-mono {
    font-family: monospace;
}

.status-bar-text-banner-small,
.status-bar-info-text {
    font-size: 12px;
    font-weight: bold;
}

.status-bar-text-banner {
    font-size: 11px;
    font-weight: bold;
}

/* # --- Colors & Utils */
.text-x {
    color: rgb(212, 58, 58);
}

.text-y {
    color: rgb(46, 177, 64);
}

.text-z {
    color: rgb(58, 61, 212);
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
    padding: 5px;
}

/* # --- Components */
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
    background-color: white;
    cursor: pointer;
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
    text-shadow: -1px -1px 0 #fff, 1px -1px 0 #fff, -1px 1px 0 #fff, 1px 1px 0 #fff;
}

.wifi-front {
    z-index: 2;
}

.wifi-backdrop {
    opacity: 0.3;
    position: absolute;
    z-index: 1;
}

.min-graph-height {
    height: 15ch;
}

/* # --- --- Layout & Scroll */
.side-menu-root {
    pointer-events: none;
    z-index: 5000;
    overflow-y: visible;
    max-height: calc(100vh - 65px);
    transition: right 0.4s ease-in-out;
}

.side-menu-root * {
    pointer-events: auto;
}

.menu {
    background: rgba(255, 255, 255, 0.65);
    border-radius: 12px;
    border: 1px solid rgba(255, 255, 255, 0.5);
    box-shadow: 0 3px 6px rgba(0, 0, 0, 0.15);
    backdrop-filter: blur(16px) saturate(150%);
    overflow-y: hidden;
}

.menu-closed {
    right: -100%;
}

.custom-scroll {
    overflow-y: auto;
    scrollbar-width: thin;
    scrollbar-color: rgba(136, 136, 136, 0.7) transparent;
}

.custom-scroll::-webkit-scrollbar {
    width: 12px;
}

.custom-scroll::-webkit-scrollbar-track {
    background: transparent;
}

.custom-scroll::-webkit-scrollbar-thumb {
    background-color: rgba(136, 136, 136, 0.7);
    border-radius: 6px;
}
</style>