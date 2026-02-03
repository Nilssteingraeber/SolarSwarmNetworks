<script setup lang="ts">
import { ref, reactive, watch, onMounted, onUnmounted, nextTick } from 'vue';
import { MDBCol, MDBRow } from 'mdb-vue-ui-kit';
import { OhVueIcon } from 'oh-vue-icons';

import TextInput from "./TextInput.vue";
import IntegerInput from './IntegerInput.vue';
import BooleanInput from './BooleanInput.vue';
import FloatInput from './FloatInput.vue';

import { UseViewedDroneStore } from '../../../stores/viewedDroneStore';
import { storeToRefs } from 'pinia';
import { DronesPollingService } from '../../../DronesData/DronesPollingService';
import { NamedParsedInterface } from '../../../DronesData/NodeRosInterfaces';
import { DataType } from '../../../models/Service';
import { useServiceInterfaceStore } from '../../../stores/ServiceInterfaceStore';

// --- Store & Refs ---
const viewedDroneStore = UseViewedDroneStore();
const { viewedNid, dronesPollingService } = storeToRefs(viewedDroneStore);
const interfaceStore = useServiceInterfaceStore();

const servicesRef = ref<NamedParsedInterface[] | undefined>(undefined);
const servicesLoading = ref(false);

const serviceInputs = reactive<Record<string, Record<string, any>>>({});
const modifiedFields = reactive(new Set<string>());

// Animation State: Ghosts
interface Ghost {
    id: string;
    style: Record<string, string>;
}
const ghosts = ref<Ghost[]>([]);

// UI State per service
const serviceUiState = reactive<Record<string, {
    showRawName: boolean,
    isScrolling: boolean,
    isLoading: boolean,     // Waiting for response
    justUpdated: boolean    // Output flash
}>>({});

// --- Time Logic ---
const currentTime = ref('');
let timeInterval: ReturnType<typeof setInterval>;

function updateTime() {
    const now = new Date();
    currentTime.value = now.toLocaleTimeString('en-US', { hour12: false });
}

onMounted(() => {
    updateTime();
    timeInterval = setInterval(updateTime, 30000);
});

onUnmounted(() => clearInterval(timeInterval));

// --- Component Map ---
const componentMap: Record<DataType, any> = {
    [DataType.Text]: TextInput,
    [DataType.Float]: FloatInput,
    [DataType.Integer]: IntegerInput,
    [DataType.Boolean]: BooleanInput,
    [DataType.Coordinates]: undefined,
    [DataType.MapCoordinates]: undefined
};

// --- Data Fetching ---
watch(viewedNid, async (nid) => {
    modifiedFields.clear();
    if (!nid || !dronesPollingService.value) {
        servicesRef.value = undefined;
        return;
    }
    const cached = interfaceStore.get(nid).value;
    if (cached && interfaceStore.has(nid).value) {
        loadServices(cached);
        return;
    }
    servicesLoading.value = true;
    try {
        const fetchedServices = await dronesPollingService.value.fetchServicesList(nid);
        interfaceStore.set(nid, fetchedServices);
        loadServices(fetchedServices);
    } catch (err) {
        console.error('Failed to fetch services:', err);
        servicesRef.value = undefined;
    } finally {
        servicesLoading.value = false;
    }
}, { immediate: true });

function loadServices(services: NamedParsedInterface[]) {
    servicesRef.value = services;
    services.forEach(svc => {
        if (!(svc.serviceName in serviceUiState)) {
            serviceUiState[svc.serviceName] = {
                showRawName: false,
                isScrolling: false,
                isLoading: false,
                justUpdated: false
            };
        }
        if (!(svc.serviceName in serviceInputs)) {
            serviceInputs[svc.serviceName] = {};
        }
        [...svc.parsed.request.inputs, ...svc.parsed.response.outputs].forEach(param => {
            if (!(param.name in serviceInputs[svc.serviceName])) {
                serviceInputs[svc.serviceName][param.name] = null;
            }
        });
    });
}

// --- Input Logic ---
function handleInputUpdate(serviceName: string, paramName: string, value: any) {
    const key = `${serviceName}::${paramName}`;
    if (value === null || value === undefined || value === '') {
        modifiedFields.delete(key);
    } else {
        modifiedFields.add(key);
    }
}

function clearInput(serviceName: string, paramName: string) {
    serviceInputs[serviceName][paramName] = null;
    modifiedFields.delete(`${serviceName}::${paramName}`);
}

function isModified(serviceName: string, paramName: string) {
    return modifiedFields.has(`${serviceName}::${paramName}`);
}

function getInputId(serviceName: string, paramName: string) {
    // Generate valid CSS ID
    return `input-${serviceName.replace(/[^a-zA-Z0-9]/g, '-')}-${paramName}`;
}

// --- Execution & Animation Logic ---

function canExecute(service: NamedParsedInterface): boolean {
    if (serviceUiState[service.serviceName]?.isLoading) return false;

    const inputs = service.parsed.request.inputs;
    if (!inputs || inputs.length === 0) return true;
    return inputs.every(param => {
        const val = serviceInputs[service.serviceName]?.[param.name];
        return val !== null && val !== undefined && val !== '';
    });
}

async function executeService(service: NamedParsedInterface, event: MouseEvent) {
    if (!canExecute(service)) return;

    const sName = service.serviceName;
    const uiState = serviceUiState[sName];
    const buttonEl = (event.currentTarget as HTMLElement);
    const buttonRect = buttonEl.getBoundingClientRect();
    const newGhosts: Ghost[] = [];

    // 1. Prepare Ghosts (Snapshot Position)
    service.parsed.request.inputs.forEach(param => {
        const elementId = getInputId(sName, param.name);
        const inputEl = document.getElementById(elementId);

        if (inputEl) {
            const rect = inputEl.getBoundingClientRect();
            newGhosts.push({
                id: Date.now() + Math.random().toString(),
                style: {
                    top: `${rect.top}px`,
                    left: `${rect.left}px`,
                    width: `${rect.width}px`,
                    height: `${rect.height}px`,
                    position: 'fixed',
                    zIndex: '9999',
                    opacity: '1',
                    transform: 'scale(1)',
                    transition: 'all 0.6s cubic-bezier(0.16, 1, 0.3, 1)', // "Suck" ease
                    pointerEvents: 'none',
                    border: '2px solid #ff34b1',
                    borderRadius: '8px',
                    backgroundColor: 'rgba(255, 255, 255, 0.4)',
                    boxSizing: 'border-box'
                }
            });
        }
    });

    // 2. Mount Ghosts to DOM
    ghosts.value = newGhosts;
    await nextTick(); // Wait for Vue to render the ghosts at start position

    // 3. Trigger Animation (Move to Button)
    requestAnimationFrame(() => {
        requestAnimationFrame(() => {
            ghosts.value.forEach(g => {
                g.style.top = `${buttonRect.top + (buttonRect.height / 2)}px`;
                g.style.left = `${buttonRect.left + (buttonRect.width / 2)}px`;
                g.style.width = '10px';
                g.style.height = '10px';
                g.style.opacity = '0';
                g.style.borderRadius = '50%';
            });
        });
    });

    // 4. Wait for animation (500ms)
    await new Promise(r => setTimeout(r, 500));

    // 5. Cleanup & Start Loading
    ghosts.value = [];
    service.parsed.request.inputs.forEach(p => clearInput(sName, p.name));
    uiState.isLoading = true;

    // 6. Mock Network Wait (2 seconds)
    await new Promise(r => setTimeout(r, 2000));

    // 7. Fill Outputs
    service.parsed.response.outputs.forEach(p => {
        if (p.type === DataType.Integer || p.type === DataType.Float) {
            serviceInputs[sName][p.name] = Math.floor(Math.random() * 100);
        } else if (p.type === DataType.Boolean) {
            serviceInputs[sName][p.name] = true;
        } else {
            serviceInputs[sName][p.name] = "Data " + new Date().getSeconds();
        }
    });

    // 8. Done
    uiState.isLoading = false;
    uiState.justUpdated = true;
    setTimeout(() => { uiState.justUpdated = false; }, 1000);
}

// --- Helpers ---
function formatParamName(name: string) {
    return name.replace(/([A-Z])/g, ' $1').replace(/^./, str => str.toUpperCase()).trim();
}

function getServiceName(service: NamedParsedInterface) {
    if (serviceUiState[service.serviceName]?.showRawName) return service.serviceName;
    return service.serviceType.split('/').pop()?.replace(/([A-Z])/g, ' $1').trim();
}

function getServiceType(service: NamedParsedInterface) {
    if (serviceUiState[service.serviceName]?.showRawName) return service.serviceType;
    return '';
}

function toggleServiceName(serviceName: string) {
    if (serviceUiState[serviceName]) {
        serviceUiState[serviceName].showRawName = !serviceUiState[serviceName].showRawName;
    }
}

function handleResize() {
    nextTick(() => {
        document.querySelectorAll<HTMLElement>('.service-name-container').forEach(el => {
            const name = el.dataset.name;
            if (name && serviceUiState[name]) {
                serviceUiState[name].isScrolling = el.scrollWidth > el.clientWidth;
            }
        });
    });
}

onMounted(() => {
    window.addEventListener('resize', handleResize);
    handleResize();
});

onUnmounted(() => window.removeEventListener('resize', handleResize));
</script>

<template>
    <MDBRow class="g-3">
        <teleport to="body">
            <div class="ghost-container">
                <div v-for="ghost in ghosts" :key="ghost.id" :style="ghost.style"></div>
            </div>
        </teleport>

        <MDBCol v-if="servicesLoading" col="12">
            <div class="loader-bar"></div>
        </MDBCol>

        <MDBCol v-else v-for="service in servicesRef" :key="service.serviceName" col="12" md="6">

            <div class="input-layer-strong p-3 h-100 d-flex flex-column">

                <div class="d-flex justify-content-between align-items-start mb-2">
                    <div class="service-name-container overflow-hidden" :data-name="service.serviceName">
                        <div class="service-name-vertical shadowy" :title="service.serviceName">
                            <span class="service-type text-truncate">{{ getServiceName(service) }}</span>
                            <span class="smallest-text text-truncate text-muted">{{ getServiceType(service) }}</span>
                        </div>
                    </div>

                    <a role="button" @click.stop="toggleServiceName(service.serviceName)" class="toggle-icon ms-2">
                        <OhVueIcon
                            :name="serviceUiState[service.serviceName]?.showRawName ? 'bi-toggle-off' : 'bi-toggle-on'"
                            scale="1.5" />
                    </a>
                </div>

                <hr class="divider opacity-10 my-2">
                <div class="loader-bar mb-3" :class="{ shrink: !servicesLoading }"></div>

                <div class="d-flex flex-wrap gap-2 mb-3 inputs-container">
                    <div v-for="param in service.parsed.request.inputs" :key="param.name"
                        :id="getInputId(service.serviceName, param.name)"
                        class="input-layer p-2 d-flex align-items-center flex-grow-1 position-relative" :class="{
                            'input-highlighted': isModified(service.serviceName, param.name)
                        }">

                        <div class="param-name white-text-outline text-nowrap">{{ formatParamName(param.name) }}</div>
                        <div class="vertical-rule mx-2"></div>

                        <component :is="componentMap[param.type as unknown as DataType] ?? TextInput"
                            v-model="serviceInputs[service.serviceName][param.name]"
                            class="flex-grow-1 input-component-fix"
                            @update:model-value="(val: any) => handleInputUpdate(service.serviceName, param.name, val)"
                            @keydown.stop @keyup.stop @keypress.stop />

                        <div v-if="isModified(service.serviceName, param.name)" class="clear-btn"
                            @click.stop="clearInput(service.serviceName, param.name)" title="Clear input">
                            <OhVueIcon name="bi-x" scale="1" />
                        </div>
                    </div>
                </div>

                <div class="d-flex align-items-center justify-content-between mb-3">

                    <div class="flex-grow-1 divider-line"></div>

                    <div class="execute-button mx-3 px-4 py-2" :class="{
                        'disabled': !canExecute(service),
                        'ready-pulse': canExecute(service) && !serviceUiState[service.serviceName]?.isLoading,
                        'loading': serviceUiState[service.serviceName]?.isLoading
                    }" @click.stop="executeService(service, $event)" title="Execute Service">

                        <template v-if="!serviceUiState[service.serviceName]?.isLoading">
                            <OhVueIcon name="fa-play" scale="1" class="me-2" />
                            <span class="fw-bold">Execute</span>
                        </template>

                        <template v-else>
                            <div class="custom-spinner me-2"></div>
                            <span class="fw-bold">Waiting...</span>
                        </template>
                    </div>

                    <div class="flex-grow-1 divider-line"></div>
                </div>

                <div v-if="service.parsed.response.outputs?.length" class="d-flex flex-wrap gap-2 mt-auto">
                    <div v-for="returnParam in service.parsed.response.outputs" :key="returnParam.name"
                        class="input-layer p-2 d-flex align-items-center flex-grow-1"
                        :class="{ 'output-flash': serviceUiState[service.serviceName]?.justUpdated }">

                        <div class="param-name white-text-outline disabled text-nowrap">{{
                            formatParamName(returnParam.name) }}</div>
                        <div class="vertical-rule mx-2"></div>

                        <component :is="componentMap[returnParam.type as unknown as DataType] ?? TextInput"
                            v-model="serviceInputs[service.serviceName][returnParam.name]" class="flex-grow-1"
                            :read-only="true" :is-output="true" />
                    </div>
                </div>

            </div>
        </MDBCol>
    </MDBRow>
</template>

<style scoped>
/* Utilities */
.cursor-pointer {
    cursor: pointer;
}

.opacity-10 {
    opacity: 0.1;
}

.input-component-fix {
    padding-right: 20px;
}

/* Horizontal Divider Lines for Action Bar */
.divider-line {
    height: 1px;
    background-color: rgba(51, 51, 51, 0.15);
    /* Subtle grey */
}

/* Structural Styling */
.input-layer-strong {
    background-color: rgba(255, 255, 255, 0.450);
    border-radius: 8px;
    border: 1px rgba(126, 126, 126, 0.178) solid;
}

.input-layer {
    background-color: rgba(255, 255, 255, 0.350);
    border-radius: 8px;
    border: 1px solid rgba(126, 126, 126, 0.178);
    transition: all 0.3s ease;
}

/* --- ANIMATIONS --- */

/* 1. Glow Cycle */
@keyframes glowCycle {
    0% {
        box-shadow: 0 0 5px rgba(255, 52, 177, 0.4);
        border-color: rgba(255, 52, 177, 0.5);
    }

    50% {
        box-shadow: 0 0 12px rgba(255, 230, 109, 0.6);
        border-color: rgba(255, 230, 109, 0.8);
    }

    100% {
        box-shadow: 0 0 5px rgba(255, 52, 177, 0.4);
        border-color: rgba(255, 52, 177, 0.5);
    }
}

.input-highlighted {
    background-color: rgba(255, 255, 255, 0.65);
    animation: glowCycle 3s infinite alternate ease-in-out;
}

/* 2. Output Flash */
@keyframes flashPop {
    0% {
        background-color: rgba(255, 52, 177, 0.1);
        transform: scale(0.98);
    }

    50% {
        background-color: rgba(255, 52, 177, 0.4);
        transform: scale(1.02);
    }

    100% {
        background-color: rgba(255, 255, 255, 0.350);
        transform: scale(1);
    }
}

.output-flash {
    animation: flashPop 0.6s ease-out;
    border-color: #ff34b1;
}

/* --- CUSTOM SPINNER --- */
.custom-spinner {
    width: 20px;
    height: 20px;
    border-radius: 50%;
    /* Creates a sweep gradient with your exact colors */
    background: conic-gradient(#ff34b1,
            #ff8abb,
            #ff9090,
            #ffe66d,
            #ff34b1);
    /* Masks out the center to make it a ring */
    -webkit-mask: radial-gradient(farthest-side, transparent calc(100% - 3px), #fff calc(100% - 3px));
    mask: radial-gradient(farthest-side, transparent calc(100% - 3px), #fff calc(100% - 3px));
    animation: spin 1s linear infinite;
}

@keyframes spin {
    0% {
        transform: rotate(0deg);
    }

    100% {
        transform: rotate(360deg);
    }
}

/* --- Execute Button --- */
.execute-button {
    border-radius: 20px;
    transition: all 0.3s ease-in-out;
    display: flex;
    align-items: center;
    justify-content: center;
    border: 1px solid transparent;
    min-width: 140px;
}

/* State: Disabled */
.execute-button.disabled {
    cursor: not-allowed;
    opacity: 0.5;
    filter: grayscale(100%);
    background-color: rgba(0, 0, 0, 0.05);
    color: #666;
}

/* State: Ready (Pulsing) */
.execute-button.ready-pulse {
    cursor: pointer;
    background-color: rgba(255, 255, 255, 0.9);
    color: rgb(46, 46, 46);
    animation: glowCycle 3s infinite alternate ease-in-out;
}

.execute-button.ready-pulse:hover {
    color: #ff34b1;
    background-color: #fff;
    transform: scale(1.05);
}

/* State: Loading */
.execute-button.loading {
    cursor: wait;
    background-color: rgba(255, 255, 255, 0.8);
    color: #ff34b1;
    border-color: #ff34b1;
}

/* --- Clear Button --- */
.clear-btn {
    position: absolute;
    right: 5px;
    top: 50%;
    transform: translateY(-50%);
    cursor: pointer;
    color: #2c2c2c;
    display: flex;
    align-items: center;
    justify-content: center;
    width: 24px;
    height: 24px;
    border-radius: 12px;
    background: rgba(255, 255, 255, 1.0);
    transition: all 0.2s;
    z-index: 10;
}

.clear-btn:hover {
    color: #ff2727;
    background: rgba(255, 199, 199, 0.9);
}

.vertical-rule {
    width: 1px;
    height: 1.5rem;
    background-color: #3333330c;
}

.shadowy {
    background: rgba(0, 0, 0, 1.0);
    background-clip: text;
    -webkit-text-fill-color: transparent;
    text-shadow: 0 1px 2px rgba(255, 255, 255, 0.3), 0 0 8px rgba(255, 255, 255, 0.5);
}

.service-type {
    font-weight: bold;
    font-size: 14px;
    display: block;
}

.disabled {
    color: rgba(46, 46, 46, 0.562);
}

.toggle-icon {
    cursor: pointer;
    transition: color 0.15s ease-in-out;
    color: #333;
}

.toggle-icon:hover {
    color: #666;
}

/* Loader */
.loader-bar {
    height: 4px;
    border-radius: 4px;
    background: linear-gradient(90deg, #ff34b1 0%, #ff8abb 15%, #ff9090 30%, #ffe66d 55%, #ff34b1 100%);
    background-size: 500% 100%;
    animation: scrollLover 1.5s linear infinite;
    transform-origin: center;
    transform: scaleY(1);
    transition: transform 0.3s ease;
}

.loader-bar.shrink {
    transform: scaleY(0);
}

@keyframes scrollLover {
    0% {
        background-position: 0 0;
    }

    100% {
        background-position: -100% 0;
    }
}

/* Ghost Container Z-Index Helper */
.ghost-container {
    position: fixed;
    top: 0;
    left: 0;
    width: 100vw;
    height: 100vh;
    pointer-events: none;
    z-index: 9999;
}
</style>