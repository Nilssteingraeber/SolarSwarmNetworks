<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount } from 'vue'
import * as Cesium from 'cesium'
import 'cesium/Build/Cesium/Widgets/widgets.css'

import { useDroneEntityStore } from '../DronesData/DroneEntityStore'
import { useDroneHistoryStore } from '../DronesData/DroneHistoryStore'
import { useDroneDataStore } from '../DronesData/DroneDataStore'

import { useSettingsStore } from '../stores/SettingsStore'
import { useTimeStore } from '../stores/TimeStore'
import { UseViewedDroneStore } from '../stores/viewedDroneStore'
import { useGeoToolsStore } from '../stores/GeoToolsStore'
import { DronesPollingService } from '../DronesData/DronesPollingService'
import { DroneSimulatorBackend } from '../DronesData/DroneSimulatorBackend'
import { FormType } from '../models/GeoForm'
import { initGeoToolsHandler } from '../components/GeoEditing/GeoToolsHandler'

// @ts-expect-error
Cesium.buildModuleUrl.setBaseUrl('/node_modules/cesium/Build/Cesium/')

// ---------------------------
// STORES
// ---------------------------
const droneEntityStore = useDroneEntityStore()
const droneHistoryStore = useDroneHistoryStore()
const droneDataStore = useDroneDataStore()
const geoToolsStore = useGeoToolsStore()

// ---------------------------
const cesiumContainer = ref(null)

let geoHandler: Cesium.ScreenSpaceEventHandler | null = null
let viewer: Cesium.Viewer | null = null
let syncInterval: number | null = null
let simTickInterval: number | null = null

// ✅ NEW: Simulator instance (shared with polling service)
let simulator: DroneSimulatorBackend | null = null


/**
 * Take a snapshot of the Cesium scene at a specific camera position and orientation.
 * @param destination Cesium.Cartesian3 - camera position
 * @param orientation {heading, pitch, roll} in radians
 * @param filename optional filename for download
 */
function takeCesiumSnapshot(
    destination: Cesium.Cartesian3,
    orientation: { heading: number; pitch: number; roll: number },
    filename = 'cesium_snapshot.png'
) {
    if (!viewer) return

    const camera = viewer.camera

    // Save current camera state
    const oldPos = camera.position.clone()
    const oldDir = camera.direction.clone()
    const oldUp = camera.up.clone()
    const oldRight = camera.right.clone()

    // Move camera to target
    camera.setView({
        destination,
        orientation
    })

    // Force a render
    viewer.render()

    // Capture canvas as PNG
    const canvas = viewer.scene.canvas
    const pngDataUrl = canvas.toDataURL('image/png')

    // Download automatically
    const a = document.createElement('a')
    a.href = pngDataUrl
    a.download = filename
    a.click()

    // Restore original camera
    camera.position = oldPos
    camera.direction = oldDir
    camera.up = oldUp
    camera.right = oldRight

    // Redraw scene
    viewer.render()
}


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
        timeline: false,
        animation: false,
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

    Cesium.CesiumTerrainProvider.fromUrl(`http://localhost:${import.meta.env.VITE_DEM_SERVER_PORT}/`)
        .then((prov) => { viewer!.scene.terrainProvider = prov })

    viewer.scene.globe.depthTestAgainstTerrain = true
    viewer.scene.debugShowFramesPerSecond = true

    const osmImageryProvider = new Cesium.OpenStreetMapImageryProvider({
        url: `http://localhost:${import.meta.env.VITE_OSM_SERVER_PORT}/styles/basic/512/`,
    })
    viewer.imageryLayers.removeAll()
    viewer.imageryLayers.add(new Cesium.ImageryLayer(osmImageryProvider))

    droneEntityStore.setViewer(viewer)

    // ---------------------------
    // Update TimeStore every tick
    // ---------------------------
    viewer.clock.onTick.addEventListener(() => {
        useTimeStore().setTime(Cesium.JulianDate.toDate(viewer!.clock.currentTime).getTime())
    })

    simulator = new DroneSimulatorBackend(3)
    const pollingService = new DronesPollingService({
        baseUrl: "http://localhost:8000",
        addRobotsBatch: useDroneHistoryStore().addRobotsBatch,
        intervalMs: 1000,
        useSimulator: false,
        droneSimulatorBackend: simulator
    })
    pollingService.start()
    UseViewedDroneStore().setDronesPollingService(pollingService)

    simTickInterval = window.setInterval(() => {
        if (simulator) {
            simulator.tick()
        }
    }, 1000)

    syncInterval = window.setInterval(() => {
        useTimeStore().setTime(Cesium.JulianDate.toDate(viewer!.clock.currentTime).getTime())
        UseViewedDroneStore().updateViewedRobot().then(() => droneEntityStore.syncEntitiesFromHistory())
    }, 500)

    // ---------------------------
    // Load 3D Tileset
    // ---------------------------
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
            if (pressed['o'])
                takeCesiumSnapshot(
                    Cesium.Cartesian3.fromDegrees(7.274741, 51.445823, 1180),
                    {
                        heading: 0,                 // 0 = north
                        pitch: Cesium.Math.toRadians(-90), // look straight down
                        roll: 0
                    },
                    'top_down_view.png'
                )

            requestAnimationFrame(updateCameraMovement)
        }
        requestAnimationFrame(updateCameraMovement)



    })

    const initData = async (viewer: Cesium.Viewer) => {

        initGeoToolsHandler(viewer)
        const savedForms = await geoToolsStore.loadFromDisk();

        savedForms.forEach((form: { data: { points: any[] }; type: any; name: any; id: any }) => {
            // We need to "Re-draw" the entities based on the saved data
            // This logic is essentially the "finalizeShape" logic but 
            // sourced from form.data.points instead of activePoints

            const points = form.data.points.map((p: { lon: number; lat: number; height: number | undefined }) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat, p.height));
            let primary: Cesium.Entity | null = null;
            let labels: Cesium.Entity[] = [];

            if (form.type === FormType.LineString) {
                primary = viewer.entities.add({
                    name: form.name,
                    polyline: { positions: points, width: 3, material: Cesium.Color.CYAN, clampToGround: true }
                });
                for (let i = 0; i < points.length - 1; i++) {
                    // Re-create label helper logic here or import it
                    const d = Cesium.Cartesian3.distance(points[i], points[i + 1]);
                    const mid = Cesium.Cartesian3.midpoint(points[i], points[i + 1], new Cesium.Cartesian3());
                    labels.push(viewer.entities.add({
                        position: mid,
                        label: { text: `${d.toFixed(2)}m`, font: "14px monospace", showBackground: true }
                    }));
                }
            }
            // ... Handle Polygon and Sphere reconstruction similarly ...

            if (primary)
                geoToolsStore.entityList.set(form.id, { primary, labels });
        });
    }

    if (viewer)
        initData(viewer)
})

</script>

<template>
    <div ref="cesiumContainer" style="width: 100%; height: 100vh"></div>
</template>
