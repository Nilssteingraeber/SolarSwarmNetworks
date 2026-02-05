<script setup lang="ts">
import { MDBRow, MDBCol, MDBBtn } from 'mdb-vue-ui-kit' // Added MDBBtn
import { OhVueIcon } from 'oh-vue-icons'
import { ref, watch } from 'vue'
import { storeToRefs } from 'pinia'
import GeoItemList from './GeoItemList.vue'
import { useDroneEntityStore } from '../../DronesData/DroneEntityStore'
import { useGeoToolsStore, SelectedTool } from '../../stores/GeoToolsStore'

// --- Main categories ---
enum MainCategory {
    None, New, Delete, Edit, List,
}

// --- Store ---
const geoStore = useGeoToolsStore()
const droneEntityStore = useDroneEntityStore()
const { isOpen } = storeToRefs(geoStore)

// --- Local state ---
const currentCategory = ref<MainCategory>(MainCategory.None)
const currentTool = ref<SelectedTool>(SelectedTool.None)

// --- Methods ---
const closeMenu = () => geoStore.setOpen(false)

const isCategorySelected = (cat: MainCategory) => currentCategory.value === cat
const isToolSelected = (tool: SelectedTool) => geoStore.activeTool === tool;

const selectCategory = (cat: MainCategory) => {
    currentCategory.value = currentCategory.value === cat ? MainCategory.None : cat
    if (currentCategory.value !== MainCategory.None) {
        currentTool.value = SelectedTool.None
    }
}

const selectTool = (tool: SelectedTool) => {
    currentTool.value = currentTool.value === tool ? SelectedTool.None : tool
}

// --- Configuration ---
interface ToolItem { name: string; icon: string; tool: SelectedTool }
interface CategoryItem { category: MainCategory; name: string; icon: string; tools?: ToolItem[] }

const categories: CategoryItem[] = [
    {
        category: MainCategory.New,
        name: 'New',
        icon: 'bi-file-earmark',
        tools: [
            { name: 'Marker', icon: 'gi-position-marker', tool: SelectedTool.Marker },
            { name: 'Plane', icon: 'bi-square-fill', tool: SelectedTool.Plane },
            { name: 'Circle', icon: 'fa-circle', tool: SelectedTool.Circle },
            { name: 'Sphere', icon: 'fa-globe', tool: SelectedTool.Sphere },
            { name: 'Line', icon: 'md-polyline', tool: SelectedTool.Line },
            { name: 'Polygon', icon: 'fa-draw-polygon', tool: SelectedTool.Polygon },
        ],
    },
    {
        category: MainCategory.Delete,
        name: 'Delete',
        icon: 'ri-delete-bin-7-line',
        // Added the tool here so it shows up in the sub-menu
        tools: [
            { name: 'Eraser', icon: 'fa-eraser', tool: SelectedTool.Delete }
        ]
    },
    {
        category: MainCategory.Edit,
        name: 'Edit',
        icon: 'bi-pencil',
        tools: [
            { name: 'Move', icon: 'bi-arrows-move', tool: SelectedTool.Move },
            { name: 'Rotate', icon: 'bi-arrow-repeat', tool: SelectedTool.Rotate },
        ],
    },
    {
        category: MainCategory.List,
        name: 'List',
        icon: 'bi-list-ul',
    },
]

// --- Watch tool selection ---
watch(currentTool, (newValue) => {
    console.log(newValue)
    geoStore.setTool(newValue)
})

// Clear tool when menu closes
watch(isOpen, (val) => {
    if (!val) currentTool.value = SelectedTool.None
})
</script>

<template>
    <MDBRow class="side-menu-root menu justify-content-end" :class="{ 'menu-closed': !isOpen }">
        <MDBCol class="col-auto text-center h-100 d-flex flex-column flex-grow-1">

            <MDBRow class="flex justify-content-between align-items-center header-row">
                <MDBCol class="col-auto title">Editing Tools</MDBCol>
                <MDBCol />
                <MDBCol class="col-auto p-2">
                    <OhVueIcon name="io-close" scale="1.5" class="input-layer close-button" @click="closeMenu" />
                </MDBCol>
            </MDBRow>

            <MDBRow class="menu-content flex-grow-1 overflow-auto custom-scroll">
                <MDBCol>
                    <MDBRow class="text-start p-2">
                        <MDBCol v-for="cat in categories" :key="cat.category"
                            class="col-auto mx-2 toggle-button rounded no-select"
                            :class="{ 'toggle-button-selected outline-animated': isCategorySelected(cat.category) }"
                            @click="selectCategory(cat.category)">
                            <MDBRow class="justify-content-center mt-2">
                                <MDBCol class="col-auto battery-center">
                                    <OhVueIcon :cat-icon="cat.icon" :name="cat.icon" scale="1.5" />
                                </MDBCol>
                            </MDBRow>
                            <MDBRow class="justify-content-center">
                                <MDBCol class="col-12">
                                    <span class="status-bar-info-text">{{ cat.name }}</span>
                                </MDBCol>
                            </MDBRow>
                        </MDBCol>
                    </MDBRow>

                    <MDBRow class="shape-select m-0 mb-3"
                        :class="{ 'shape-select-closed': !isCategorySelected(currentCategory) }">

                        <MDBCol v-for="tool in categories.find(c => c.category === currentCategory)?.tools || []"
                            :key="tool.tool" class="col-auto rounded toggle-button-small m-2"
                            :class="{ 'toggle-button-selected outline-animated': isToolSelected(tool.tool) }"
                            @click="selectTool(tool.tool)">
                            <MDBRow class="justify-content-center">
                                <MDBCol class="col-12">
                                    <span class="status-bar-info-text">{{ tool.name }}</span>
                                </MDBCol>
                            </MDBRow>
                            <MDBRow class="justify-content-center">
                                <MDBCol class="col-auto battery-center">
                                    <OhVueIcon :name="tool.icon" scale="1.0" />
                                </MDBCol>
                            </MDBRow>
                        </MDBCol>

                        <MDBCol v-if="isCategorySelected(MainCategory.List)" class="col-12">
                            <GeoItemList />
                        </MDBCol>
                    </MDBRow>
                </MDBCol>
            </MDBRow>

            <MDBRow class="p-2 mt-auto rounded-bottom">
                <MDBCol class="d-flex justify-content-end">
                    <MDBBtn size="sm" color="danger"
                        @click="droneEntityStore.viewer && geoStore.clearAll(droneEntityStore.viewer)">
                        Reset DB
                    </MDBBtn>
                    <MDBBtn size="sm" color="primary" @click="geoStore.exportToJson()">
                        Export JSON
                    </MDBBtn>
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
    src: url('@/assets/latinmodern-math.otf') format('opentype');
}

.text-x {
    color: rgb(212, 58, 58);
}

.text-y {
    color: rgb(46, 177, 64);
}

.text-z {
    color: rgb(58, 61, 212);
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
    max-height: calc(100vh - 65px);
    overflow-y: visible;
    width: 100%;
    max-width: 100%;
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
    text-shadow:
        -1px -1px 0 #ffffff,
        1px -1px 0 #ffffff,
        -1px 1px 0 #ffffff,
        1px 1px 0 #ffffff;
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
    background-color: rgba(255, 255, 255, 0.35);
    border-radius: 8px;
    border: 1px rgba(126, 126, 126, 0.178) solid;
}

.input-layer {
    background-color: rgba(255, 255, 255, 0.35);
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
    background: rgba(0, 0, 0, 1);
    background-clip: text;
    -webkit-text-fill-color: transparent;

    text-shadow:
        0 1px 2px rgba(255, 255, 255, 0.3),
        0 0 8px rgba(255, 255, 255, 0.5);
}

.title {
    font-family: Verdana, Geneva, Tahoma, sans-serif;
    font-size: xx-large;

    background: rgba(0, 0, 0, 1);
    background-clip: text;
    -webkit-text-fill-color: transparent;

    text-shadow:
        0 1px 2px rgba(255, 255, 255, 0.3),
        0 0 8px rgba(255, 255, 255, 0.5);
}

.sticky {
    position: sticky;
    /* makes it sticky */
    top: 0px;
    /* distance from the top of the scroll container */
    z-index: 2000;
    /* semi-transparent white */
    backdrop-filter: blur(1px);
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

@keyframes dash {
    to {
        stroke-dashoffset: -10;
    }
}

.toggle-button-selected {
    color: rgb(29, 132, 228);
    outline: solid 1px rgba(228, 228, 228, 0.936);
    box-shadow: rgba(83, 83, 83, 0.329) 0px 3px 4px;
    background-color: #ffffff44;
}

.toggle-button {
    transition: all 0.3s ease;
}

.toggle-button:hover {
    color: rgb(51, 157, 255);
    cursor: pointer;
}

.toggle-button-small-selected {
    color: rgb(29, 132, 228);

}

.toggle-button-small {
    transition: all 0.3s ease;
}

.toggle-button-small:hover {
    color: rgb(30, 146, 255);
    cursor: pointer;
}

.shape-select {
    transition: all 0.3s ease;
}

.shape-select-opened {
    height: 100vh;
    opacity: 1;
}

.shape-select-closed {
    height: 0;
    opacity: 0;
    overflow: hidden;
}

.no-select {
    user-select: none;
    -webkit-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
}

.menu-closed {
    right: -100%;
    /* move completely offscreen */
}
</style>
