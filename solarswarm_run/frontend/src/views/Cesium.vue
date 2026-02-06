<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount } from 'vue'
import * as Cesium from 'cesium'
import 'cesium/Build/Cesium/Widgets/widgets.css'

// # --- Imports: Stores & Data
import { useDroneEntityStore } from '../dronesData/DroneEntityStore'
import { useDroneHistoryStore } from '../dronesData/DroneHistoryStore'
import { useDroneDataStore } from '../dronesData/DroneDataStore'
import { useSettingsStore } from '../stores/SettingsStore'
import { useTimeStore } from '../stores/TimeStore'
import { UseViewedDroneStore } from '../stores/viewedDroneStore'
import { useGeoToolsStore } from '../stores/GeoToolsStore'
import { DronesPollingService } from '../dronesData/DronesPollingService'
import { DroneSimulatorBackend } from '../dronesData/DroneSimulatorBackend'
import { FormType } from '../models/GeoForm'
import { initGeoToolsHandler } from '../components/GeoEditing/GeoToolsHandler'

// @ts-expect-error
Cesium.buildModuleUrl.setBaseUrl('/node_modules/cesium/Build/Cesium/')

// # --- Stores
const droneEntityStore = useDroneEntityStore()
const droneHistoryStore = useDroneHistoryStore()
const geoToolsStore = useGeoToolsStore()
const timeStore = useTimeStore()
const viewedDroneStore = UseViewedDroneStore()

// # --- --- State & Refs
const cesiumContainer = ref<HTMLElement | null>(null)
let viewer: Cesium.Viewer | null = null
let geoHandler: Cesium.ScreenSpaceEventHandler | null = null
let simulator: DroneSimulatorBackend | null = null

// # --- Timers
let syncInterval: number | null = null
let simTickInterval: number | null = null
let cameraAnimationFrame: number | null = null

// # --- Input State
const pressedKeys: Record<string, boolean> = {}

// # --- Methods: Camera & Input
const handleKeyDown = (e: KeyboardEvent) => { pressedKeys[e.key.toLowerCase()] = true }
const handleKeyUp = (e: KeyboardEvent) => { pressedKeys[e.key.toLowerCase()] = false }

function updateCameraMovement() {
    if (!viewer) return

    // @ts-check
    if (true) return;

    const camera = viewer.camera
    const moveSpeed = 2
    const moveSpeedFast = 25
    const speed = pressedKeys['shift'] ? moveSpeedFast : moveSpeed

    if (pressedKeys['w']) camera.moveForward(speed)
    if (pressedKeys['s']) camera.moveBackward(speed)
    if (pressedKeys['a']) camera.moveLeft(speed)
    if (pressedKeys['d']) camera.moveRight(speed)
    if (pressedKeys[' ']) camera.moveUp(speed)
    if (pressedKeys['control']) camera.moveDown(speed)

    if (pressedKeys['o']) {
        pressedKeys['o'] = false // Debounce immediately
        takeCesiumSnapshot(
            Cesium.Cartesian3.fromDegrees(7.274741, 51.445823, 1180),
            { heading: 0, pitch: Cesium.Math.toRadians(-90), roll: 0 },
            'top_down_view.png'
        )
    }

    cameraAnimationFrame = requestAnimationFrame(updateCameraMovement)
}

function takeCesiumSnapshot(
    destination: Cesium.Cartesian3,
    orientation: { heading: number; pitch: number; roll: number },
    filename = 'cesium_snapshot.png'
) {
    if (!viewer) return

    const camera = viewer.camera
    const oldState = {
        pos: camera.position.clone(),
        dir: camera.direction.clone(),
        up: camera.up.clone(),
        right: camera.right.clone()
    }

    camera.setView({ destination, orientation })
    viewer.render()

    const link = document.createElement('a')
    link.href = viewer.scene.canvas.toDataURL('image/png')
    link.download = filename
    link.click()

    // Restore Camera
    camera.position = oldState.pos
    camera.direction = oldState.dir
    camera.up = oldState.up
    camera.right = oldState.right
    viewer.render()
}

// # --- Methods: Geo Tools
async function restoreSavedGeoForms(v: Cesium.Viewer) {
    initGeoToolsHandler(v)
    const savedForms = await geoToolsStore.loadFromDisk()

    savedForms.forEach((form: any) => {
        const points = form.data.points.map((p: any) =>
            Cesium.Cartesian3.fromDegrees(p.lon, p.lat, p.height)
        )

        let primary: Cesium.Entity | null = null
        let labels: Cesium.Entity[] = []

        if (form.type === FormType.LineString) {
            primary = v.entities.add({
                name: form.name,
                polyline: {
                    positions: points,
                    width: 3,
                    material: Cesium.Color.CYAN,
                    clampToGround: true
                }
            })

            for (let i = 0; i < points.length - 1; i++) {
                const dist = Cesium.Cartesian3.distance(points[i], points[i + 1])
                const mid = Cesium.Cartesian3.midpoint(points[i], points[i + 1], new Cesium.Cartesian3())
                labels.push(v.entities.add({
                    position: mid,
                    label: {
                        text: `${dist.toFixed(2)}m`,
                        font: "14px monospace",
                        showBackground: true
                    }
                }))
            }
        }

        // @ts-check
        // TODO

        if (primary) {
            geoToolsStore.entityList.set(form.id, { primary, labels })
        }
    })
}

// # --- Lifecycle: Mount
onMounted(async () => {
    // @ts-expect-error
    viewer = new Cesium.Viewer(cesiumContainer.value, {
        baseLayerPicker: false, timeline: false, animation: false,
        homeButton: false, vrButton: false, navigationHelpButton: false,
        fullscreenButton: false, sceneModePicker: false, geocoder: false,
        creditContainer: document.createElement('div'),
        requestRenderMode: true, shouldAnimate: true,
    })

    // 2. Terrain & Imagery
    try {
        // TERRAIN
        const terrainProvider = await Cesium.CesiumTerrainProvider.fromUrl(`http://localhost:${import.meta.env.VITE_DEM_SERVER_PORT}/`)
        viewer.scene.terrainProvider = terrainProvider
    } catch (e) { console.warn("Terrain load failed", e) }

    viewer.scene.globe.depthTestAgainstTerrain = true
    viewer.scene.debugShowFramesPerSecond = true

    // OSM
    viewer.imageryLayers.removeAll()
    viewer.imageryLayers.add(new Cesium.ImageryLayer(
        new Cesium.OpenStreetMapImageryProvider({
            url: `http://localhost:${import.meta.env.VITE_OSM_SERVER_PORT}/styles/basic/512/`,
        })
    ))

    droneEntityStore.setViewer(viewer)

    // 3D-MESH
    Cesium.Cesium3DTileset.fromUrl(`http://localhost:${import.meta.env.VITE_MESH_3D_SERVER_PORT}/tileset.json`, {
        dynamicScreenSpaceError: true,
        dynamicScreenSpaceErrorDensity: 0.00278,
        dynamicScreenSpaceErrorFactor: 8.0,
        dynamicScreenSpaceErrorHeightFalloff: 0.4,
    }).then((tileset) => {
        if (!viewer) return
        viewer.scene.primitives.add(tileset)
        useSettingsStore().setTileset(tileset)
    })

    // Viewer Settings
    viewer.targetFrameRate = 120
    viewer.clock.clockRange = Cesium.ClockRange.UNBOUNDED
    viewer.cesiumWidget.screenSpaceEventHandler.removeInputAction(Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK)
    viewer.cesiumWidget.screenSpaceEventHandler.removeInputAction(Cesium.ScreenSpaceEventType.LEFT_CLICK)

    // Time- Syncing
    viewer.clock.onTick.addEventListener(() => {
        useTimeStore().setTime(Cesium.JulianDate.toDate(viewer!.clock.currentTime).getTime())
    })

    // Backend Services
    simulator = new DroneSimulatorBackend(3)
    const pollingService = new DronesPollingService({
        baseUrl: "http://localhost:8000",
        addRobotsBatch: useDroneHistoryStore().addRobotsBatch,
        intervalMs: 1000,
        useSimulator: false,
        droneSimulatorBackend: simulator
    })
    pollingService.start()
    viewedDroneStore.setDronesPollingService(pollingService)

    // Intervals u. Loops
    simTickInterval = window.setInterval(() => simulator?.tick(), 1000)

    syncInterval = window.setInterval(() => {
        if (!viewer) return
        timeStore.setTime(Cesium.JulianDate.toDate(viewer.clock.currentTime).getTime())
        viewedDroneStore.updateViewedRobot().then(() => droneEntityStore.syncEntitiesFromHistory())
    }, 500)

    window.addEventListener('keydown', handleKeyDown)
    window.addEventListener('keyup', handleKeyUp)
    updateCameraMovement()

    const centerNRW = Cesium.Cartesian3.fromDegrees(7.5, 51.433, 81000)
    viewer.camera.setView({
        destination: centerNRW,
        orientation: { heading: 0, pitch: Cesium.Math.toRadians(-30), roll: 0 }
    })

    viewer.scene.camera.flyTo({
        destination: Cesium.Cartesian3.fromDegrees(7.277141, 51.450823, 1180),
        duration: 1.5,
    })

    await restoreSavedGeoForms(viewer)
})

onBeforeUnmount(() => {
    window.removeEventListener('keydown', handleKeyDown)
    window.removeEventListener('keyup', handleKeyUp)
    if (cameraAnimationFrame) cancelAnimationFrame(cameraAnimationFrame)

    if (syncInterval) clearInterval(syncInterval)
    if (simTickInterval) clearInterval(simTickInterval)

    if (geoHandler) geoHandler.destroy()
    if (viewer) viewer.destroy()

    viewer = null
    geoHandler = null
})
</script>

<template>
    <div ref="cesiumContainer" style="width: 100%; height: 100vh"></div>
</template>