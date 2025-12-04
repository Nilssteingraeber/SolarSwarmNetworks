<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount } from 'vue'
import * as Cesium from 'cesium'
import 'cesium/Build/Cesium/Widgets/widgets.css'

import { useDroneEntityStore } from '../DronesData/DroneEntityStore'
import { useDroneHistoryStore } from '../DronesData/DroneHistoryStore'
import { useDroneDataStore } from '../DronesData/DroneDataStore'

import { initGeoToolsHandler } from "../components/GeoEditing/GeoTools"
import { useSettingsStore } from '../stores/SettingsStore'
import { useTimeStore } from '../stores/TimeStore'
import { UseViewedDroneStore } from '../stores/viewedDroneStore'
import { DronesPollingService } from '../DronesData/DronesPollingService'
import { DroneSimulatorBackend } from '../DronesData/DroneSimulatorBackend'

// @ts-expect-error
Cesium.buildModuleUrl.setBaseUrl('/node_modules/cesium/Build/Cesium/')

// ---------------------------
// STORES
// ---------------------------
const droneEntityStore = useDroneEntityStore()
const droneHistoryStore = useDroneHistoryStore()
const droneDataStore = useDroneDataStore()

// ---------------------------
const cesiumContainer = ref(null)

let geoHandler: Cesium.ScreenSpaceEventHandler | null = null
let viewer: Cesium.Viewer | null = null
let syncInterval: number | null = null
let simTickInterval: number | null = null

// ✅ NEW: Simulator instance (shared with polling service)
let simulator: DroneSimulatorBackend | null = null

// ---------------------------
// CLEANUP
// ---------------------------
onBeforeUnmount(() => {
    if (geoHandler) {
        try { geoHandler.destroy() } catch { }
        geoHandler = null
    }
    if (viewer) {
        try { viewer.destroy() } catch { }
        viewer = null
    }
    if (syncInterval !== null) {
        clearInterval(syncInterval)
        syncInterval = null
    }
    if (simTickInterval !== null) {
        clearInterval(simTickInterval)
        simTickInterval = null
    }
})

// ---------------------------
// INIT
// ---------------------------
onMounted(() => {
    // @ts-expect-error
    viewer = new Cesium.Viewer(cesiumContainer.value, {
        baseLayerPicker: false,
        timeline: true,
        animation: true,
        homeButton: false,
        vrButton: false,
        navigationHelpButton: false,
        fullscreenButton: false,
        sceneModePicker: false,
        creditContainer: document.createElement('div'),
        requestRenderMode: true,
        shouldAnimate: true,
        geocoder: false,
    })

    // @ts-expect-error
    Cesium.CesiumTerrainProvider.fromUrl(`http://localhost:${import.meta.env.VITE_DEM_SERVER_PORT}/`)
        .then((prov) => { viewer!.scene.terrainProvider = prov })

    viewer.scene.globe.depthTestAgainstTerrain = true
    viewer.scene.debugShowFramesPerSecond = true

    const osmImageryProvider = new Cesium.OpenStreetMapImageryProvider({
        // @ts-expect-error
        url: `http://localhost:${import.meta.env.VITE_OSM_SERVER_PORT}/styles/basic/512/`,
    })
    viewer.imageryLayers.removeAll()
    viewer.imageryLayers.add(new Cesium.ImageryLayer(osmImageryProvider))

    droneEntityStore.setViewer(viewer)
    geoHandler = initGeoToolsHandler(viewer)

    // ---------------------------
    // Update TimeStore every tick
    // ---------------------------
    viewer.clock.onTick.addEventListener(() => {
        useTimeStore().setTime(Cesium.JulianDate.toDate(viewer!.clock.currentTime).getTime())
    })

    simulator = new DroneSimulatorBackend(3)
    const pollingService = new DronesPollingService({
        // @ts-expect-error
        baseUrl: import.meta.env.VITE_USE_SIM === 'true'
            ? 'http://localhost:3001'  // ignored when using simulator
            // @ts-expect-error
            : import.meta.env.DB_URL,
        addRobotsBatch: useDroneHistoryStore().addRobotsBatch,
        intervalMs: 1000,
        // @ts-expect-error
        useSimulator: import.meta.env.VITE_USE_SIM === 'true',
        droneSimulatorBackend: simulator
    })
    pollingService.start()

    simTickInterval = window.setInterval(() => {
        if (simulator) {
            simulator.tick()
        }
    }, 1000)

    syncInterval = window.setInterval(() => {
        useTimeStore().setTime(Cesium.JulianDate.toDate(viewer!.clock.currentTime).getTime())
        UseViewedDroneStore().updateViewedRobot().then(() => droneEntityStore.syncEntitiesFromHistory())
    }, 1000)

    // ---------------------------
    // Load 3D Tileset
    // ---------------------------
    // @ts-expect-error
    Cesium.Cesium3DTileset.fromUrl(`http://localhost:${import.meta.env.VITE_MESH_3D_SERVER_PORT}/tileset.json`, {
        dynamicScreenSpaceError: true,
        dynamicScreenSpaceErrorDensity: 0.00278,
        dynamicScreenSpaceErrorFactor: 8.0,
        dynamicScreenSpaceErrorHeightFalloff: 0.4,
    }).then((tileset) => {
        viewer!.scene.primitives.add(tileset)
        viewer!.targetFrameRate = 120

        viewer!.clock.clockRange = Cesium.ClockRange.UNBOUNDED

        viewer!.cesiumWidget.screenSpaceEventHandler.removeInputAction(Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK)
        viewer!.cesiumWidget.screenSpaceEventHandler.removeInputAction(Cesium.ScreenSpaceEventType.LEFT_CLICK)
        viewer!.clock.shouldAnimate = true

        useSettingsStore().setTileset(tileset)

        const now = Cesium.JulianDate.now()
        const startTime = Cesium.JulianDate.addSeconds(now, -600, new Cesium.JulianDate()) // 10 min past
        const stopTime = Cesium.JulianDate.addSeconds(now, 7200, new Cesium.JulianDate())  // 2 hours future
        viewer!.clock.startTime = startTime
        viewer!.clock.stopTime = stopTime
        viewer!.clock.currentTime = now
        viewer!.timeline.zoomTo(startTime, stopTime)

        // Initial Camera View
        const centerNRW = Cesium.Cartesian3.fromDegrees(7.5, 51.433, 81000)
        viewer!.camera.setView({
            destination: centerNRW,
            orientation: { heading: 0, pitch: Cesium.Math.toRadians(-30), roll: 0 }
        })

        // Fly to Tileset Area
        viewer!.scene.camera.flyTo({
            destination: Cesium.Cartesian3.fromDegrees(7.274741, 51.445823, 1180),
            duration: 1.5,
        })

        // ---------------------------
        // WASD Camera Movement
        // ---------------------------
        const pressed: Record<string, boolean> = {}
        const moveSpeed = 2
        const moveSpeedFast = 25

        window.addEventListener('keydown', e => pressed[e.key.toLowerCase()] = true)
        window.addEventListener('keyup', e => pressed[e.key.toLowerCase()] = false)

        const updateCameraMovement = () => {
            if (!viewer) return
            const camera = viewer.camera
            const speed = pressed['shift'] ? moveSpeedFast : moveSpeed

            if (pressed['w']) camera.moveForward(speed)
            if (pressed['s']) camera.moveBackward(speed)
            if (pressed['a']) camera.moveLeft(speed)
            if (pressed['d']) camera.moveRight(speed)
            if (pressed[' ']) camera.moveUp(speed)
            if (pressed['control']) camera.moveDown(speed)

            requestAnimationFrame(updateCameraMovement)
        }
        requestAnimationFrame(updateCameraMovement)
    })
})
</script>

<template>
    <div ref="cesiumContainer" style="width: 100%; height: 100vh"></div>
</template>
