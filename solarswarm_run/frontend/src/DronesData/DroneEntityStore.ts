import droneModel from '../assets/Robots/Drone.glb?url'
import nucModel from '../assets/Robots/NUC.glb?url'

import { defineStore } from 'pinia'
import { markRaw } from 'vue'
import {
    Entity,
    Cartesian3,
    ConstantProperty,
    SampledPositionProperty,
    JulianDate,
    VelocityOrientationProperty,
    Color,
    LabelStyle,
    VerticalOrigin,
    CallbackProperty,
    Cartographic,
    Cartesian2,
    Viewer,
    ExtrapolationType,
    PolylineDashMaterialProperty,
    // Primitive API imports
    GeometryInstance,
    PolylineGeometry,
    PolylineColorAppearance,
    PointPrimitiveCollection,
    Primitive
} from 'cesium'

import { useDroneHistoryStore } from './DroneHistoryStore'
import { useTimeStore } from '../stores/TimeStore'
import type { Robot, RobotWithEntity } from '../models/Robot'

const getDroneColor = (robot: Robot, currentTime: number) => {
    const lastHeard = robot.last_heard || 0
    const timeSinceLast = (currentTime - (lastHeard * 1000.0)) / 1000.0 / 1000.0

    if (timeSinceLast < 5) return Color.GREEN
    if (timeSinceLast < 7) return Color.YELLOW
    if (timeSinceLast < 9) return Color.RED
    return Color.GRAY
}

export const useDroneEntityStore = defineStore('droneEntities', {
    state: () => ({
        viewer: null as Viewer | null,
        entities: markRaw(new Map<string, Entity>()),
        robots: markRaw(new Map<string, Robot>()),
        connectionLines: markRaw(new Map<string, Entity>()),

        // High-performance primitives
        _flightPathPrimitives: markRaw(new Map<string, Primitive>()),
        _hoverPointCollections: markRaw(new Map<string, PointPrimitiveCollection>())
    }),

    getters: {
        getRobots(state): RobotWithEntity[] {
            const list: RobotWithEntity[] = []
            state.robots.forEach((robot, nid) => {
                const ent = state.entities.get(nid)
                if (ent) list.push({ robot, entity: ent })
            })
            return list
        },
        getRobot(state) {
            return (nid: string): RobotWithEntity | undefined => {
                const robot = state.robots.get(nid)
                const ent = state.entities.get(nid)
                if (!robot || !ent) return undefined
                return { robot, entity: ent }
            }
        }
    },

    actions: {
        setViewer(viewer: Viewer) {
            this.viewer = markRaw(viewer)
        },

        async syncEntitiesFromHistory() {
            if (!this.viewer) return
            const historyStore = useDroneHistoryStore()
            const timeStore = useTimeStore()
            const currentTimestamp = timeStore.currentTime
            const windowMs = 5000

            const dronesMap = await historyStore.getDronesInWindow(currentTimestamp, windowMs)

            this.viewer.entities.suspendEvents()

            const updatePromises = Array.from(dronesMap.entries()).map(async ([nid, robot]) => {
                this.robots.set(nid, robot)
                // Also ensure history is primed so we can draw paths immediately if selected
                const entry = await historyStore.getSnapshotAt(nid, currentTimestamp, windowMs)

                const useTime = entry
                    ? JulianDate.fromDate(new Date(entry.timestamp))
                    : JulianDate.fromDate(new Date(currentTimestamp))

                if (!this.entities.has(nid)) {
                    if (robot.point) this._createEntity(robot, useTime)
                } else {
                    this._updateEntity(robot, useTime)
                }
            })

            await Promise.all(updatePromises)
            this.viewer.entities.resumeEvents()
        },

        async drawFlightPath(nid: string, aroundTime?: number, entryCount: number = 50) {
            if (!this.viewer) return
            const historyStore = useDroneHistoryStore()
            const timeStore = useTimeStore()

            // 1. Determine Time Window (TRAIL MODE: Past -> Now)
            const currentTime = aroundTime ?? timeStore.currentTime
            const windowMs = entryCount * 1000

            // Load slightly more data than needed to ensure we have the start point
            await historyStore.ensureDroneDataLoaded(nid, currentTime, windowMs + 5000)

            const history = historyStore.cache.get(nid)
            if (!history || history.length === 0) return

            // 2. Slice History (Trail Logic: [Now - Window] to [Now])
            const startTs = currentTime - windowMs
            const endTs = currentTime // Don't show future points in trail mode

            let startIdx = 0, endIdx = history.length - 1
            while (startIdx < history.length && history[startIdx].timestamp < startTs) startIdx++
            while (endIdx >= 0 && history[endIdx].timestamp > endTs) endIdx--

            const entries = history.slice(startIdx, endIdx + 1)

            // Need at least 1 point to draw a line to the drone
            if (entries.length < 1) return

            // --- CLEANUP ---
            this.removeFlightPathLines(nid)

            // --- BUILD GEOMETRY ---
            const positions: Cartesian3[] = []
            const colors: Color[] = []
            const hoverPoints: Cartesian3[] = []

            const HOVER_RADIUS = 0.25
            const MIN_HOVER_DURATION_MS = 2000
            let clusterStartIdx = 0
            let clusterCenter: Cartesian3 | null = null

            // 3. Process History Points
            for (let i = 0; i < entries.length; i++) {
                const p = entries[i].data.point
                if (!p) continue

                const pos = Cartesian3.fromDegrees(p.lon, p.lat, p.alt ?? 150)
                positions.push(pos)

                // --- GRADIENT COLOR ---
                // 0.0 (Oldest) -> 1.0 (Latest History Point)
                const progress = i / (entries.length > 1 ? entries.length - 1 : 1)

                // Hue: Blue (0.65) -> Red (0.0)
                const hue = 0.65 * (1.0 - progress)
                // Lightness: Fades tail to black (0.0 -> 0.5)
                const lightness = 0.5 * Math.pow(progress, 0.4)

                colors.push(Color.fromHsl(hue, 1.0, lightness, 1.0))

                // --- HOVER LOGIC ---
                if (i === 0) {
                    clusterCenter = pos
                    clusterStartIdx = 0
                } else if (clusterCenter) {
                    const dist = Cartesian3.distance(pos, clusterCenter)
                    if (dist > HOVER_RADIUS) {
                        const duration = entries[i - 1].timestamp - entries[clusterStartIdx].timestamp
                        if (duration > MIN_HOVER_DURATION_MS) {
                            hoverPoints.push(clusterCenter)
                        }
                        clusterStartIdx = i
                        clusterCenter = pos
                    }
                }
            }

            // 4. [CRITICAL FIX] Add the "Connector" Point
            // The history is discrete (e.g. every 100ms). The drone is interpolated (e.g. +50ms).
            // We must add the drone's *exact* current position to the line so it touches the model.
            const entity = this.entities.get(nid)
            if (entity && entity.position) {
                const nowJulian = JulianDate.fromDate(new Date(currentTime))
                const currentPos = entity.position.getValue(nowJulian)

                if (currentPos) {
                    const lastHistPos = positions[positions.length - 1]
                    // Only add if we have moved slightly from the last history tick
                    if (Cartesian3.distance(currentPos, lastHistPos) > 0.01) {
                        positions.push(currentPos)
                        // The tip of the line is fully Red and Bright
                        colors.push(Color.fromHsl(0.0, 1.0, 0.5, 1.0))
                    }
                }
            }

            // --- RENDER PATH ---
            if (positions.length >= 2) {
                const linePrimitive = this.viewer.scene.primitives.add(new Primitive({
                    geometryInstances: new GeometryInstance({
                        geometry: new PolylineGeometry({
                            positions: positions,
                            width: 4.0,
                            vertexFormat: PolylineColorAppearance.VERTEX_FORMAT,
                            colors: colors,
                            colorsPerVertex: true
                        })
                    }),
                    appearance: new PolylineColorAppearance({ translucent: false }),
                    asynchronous: false
                }))
                this._flightPathPrimitives.set(nid, markRaw(linePrimitive))
            }

            // --- RENDER HOVER POINTS ---
            if (hoverPoints.length > 0) {
                const pointCollection = this.viewer.scene.primitives.add(new PointPrimitiveCollection())
                hoverPoints.forEach(pt => {
                    pointCollection.add({
                        position: pt,
                        color: Color.WHITE.withAlpha(0.9),
                        outlineColor: Color.BLACK,
                        outlineWidth: 2,
                        pixelSize: 6,
                        disableDepthTestDistance: Number.POSITIVE_INFINITY
                    })
                })
                this._hoverPointCollections.set(nid, markRaw(pointCollection))
            }
        },

        removeFlightPathLines(nid: string) {
            if (!this.viewer) return

            const linePrim = this._flightPathPrimitives.get(nid)
            if (linePrim) {
                this.viewer.scene.primitives.remove(linePrim)
                this._flightPathPrimitives.delete(nid)
            }

            const pointColl = this._hoverPointCollections.get(nid)
            if (pointColl) {
                this.viewer.scene.primitives.remove(pointColl)
                this._hoverPointCollections.delete(nid)
            }
        },

        _createEntity(robot: Robot, recordedTime: JulianDate) {
            if (!this.viewer || !robot.point) return

            const { lon, lat, alt } = robot.point
            const pos = Cartesian3.fromDegrees(lon, lat, alt ?? 150)

            const sampled = new SampledPositionProperty()
            sampled.forwardExtrapolationType = ExtrapolationType.HOLD
            sampled.backwardExtrapolationType = ExtrapolationType.HOLD
            sampled.addSample(recordedTime, pos)

            const ent = this.viewer.entities.add({
                id: robot.nid,
                position: sampled,
                orientation: new VelocityOrientationProperty(sampled),
                model: { uri: nucModel, scale: 1.2 },
                label: {
                    text: robot.display_name ?? robot.nid,
                    font: '14px monospace',
                    style: LabelStyle.FILL_AND_OUTLINE,
                    outlineWidth: 3,
                    verticalOrigin: VerticalOrigin.BOTTOM,
                    pixelOffset: new Cartesian2(0, -25),
                    disableDepthTestDistance: Number.POSITIVE_INFINITY
                },
                point: {
                    pixelSize: 12,
                    color: new CallbackProperty(() => getDroneColor(robot, useTimeStore().currentTime), false),
                    outlineColor: Color.BLACK,
                    outlineWidth: 2,
                    disableDepthTestDistance: Number.POSITIVE_INFINITY
                },
                properties: {
                    isSelectable: new ConstantProperty(true),
                    droneData: new ConstantProperty(robot)
                }
            })
            this.entities.set(robot.nid, markRaw(ent))

            const lineEnt = this.viewer.entities.add({
                id: `${robot.nid}-line`,
                polyline: {
                    positions: new CallbackProperty((time) => {
                        const dronePos = sampled.getValue(time)
                        if (!dronePos) return []
                        const carto = Cartographic.fromCartesian(dronePos)
                        const groundPos = Cartesian3.fromRadians(carto.longitude, carto.latitude, 0)
                        return [dronePos, groundPos]
                    }, false),
                    width: 2,
                    material: new PolylineDashMaterialProperty({
                        color: Color.YELLOW.withAlpha(0.6),
                        dashLength: 8
                    })
                }
            })
            this.connectionLines.set(`${robot.nid}-line`, markRaw(lineEnt))
        },

        _updateEntity(robot: Robot, recordedTime: JulianDate) {
            const ent = this.entities.get(robot.nid)
            if (!ent || !robot.point) return

            const { lon, lat, alt } = robot.point
            const newPos = Cartesian3.fromDegrees(lon, lat, alt ?? 150)

            if (ent.position instanceof SampledPositionProperty) {
                ent.position.addSample(recordedTime, newPos)
            }
            this.robots.set(robot.nid, robot)
            if (ent.properties && ent.properties.droneData) {
                ent.properties.droneData.setValue(robot)
            }
        },

        clear() {
            if (this.viewer) {
                this.entities.forEach(ent => this.viewer!.entities.remove(ent))
                this.connectionLines.forEach(line => this.viewer!.entities.remove(line))
                this._flightPathPrimitives.forEach(prim => this.viewer!.scene.primitives.remove(prim))
                this._hoverPointCollections.forEach(coll => this.viewer!.scene.primitives.remove(coll))
            }
            this.entities.clear()
            this.robots.clear()
            this.connectionLines.clear()
            this._flightPathPrimitives.clear()
            this._hoverPointCollections.clear()
        }
    }
})