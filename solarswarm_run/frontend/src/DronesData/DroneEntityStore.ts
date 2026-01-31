// @ts-expect-error
import droneModel from '../assets/Robots/Drone.glb?url'
// @ts-expect-error
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
    PolylineOutlineMaterialProperty,
    CallbackProperty,
    Cartographic,
    Ellipsoid,
    Cartesian2,
    Viewer,
    ExtrapolationType,
    PolylineDashMaterialProperty
} from 'cesium'

import { useDroneHistoryStore } from './DroneHistoryStore'
import { useTimeStore } from '../stores/TimeStore'
import type { Robot, RobotWithEntity } from '../models/Robot'

const getDroneColor = (robot: Robot) => {
    const timeStore = useTimeStore()
    const now = timeStore.currentTime
    const lastHeard = robot.last_heard || 0
    // Difference in seconds
    const timeSinceLast = (now - (lastHeard * 1000)) / 1000

    if (timeSinceLast < 5) return Color.GREEN
    if (timeSinceLast < 7) return Color.YELLOW
    if (timeSinceLast < 9) return Color.RED
    return Color.GRAY
}

export const useDroneEntityStore = defineStore('droneEntities', {
    state: () => ({
        viewer: null as Viewer | null,
        entities: new Map<string, Entity>(),
        robots: new Map<string, Robot>(),
        connectionLines: new Map<string, Entity>(),
        _flightPathCache: new Map<string, { start: number; end: number }[]>(),
        _tickHandler: undefined as ((clock: any) => void) | undefined
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
            const windowMs = 2500

            // 1. Get snapshot of all drones active in the current window
            const dronesMap = await historyStore.getDronesInWindow(currentTimestamp, windowMs)

            for (const [nid, robot] of dronesMap.entries()) {
                this.robots.set(nid, robot)

                // 2. Get the specific history entry for exact sub-second timestamping
                const entry = await historyStore.getSnapshotAt(nid, currentTimestamp, windowMs)
                const useTime = entry ? JulianDate.fromDate(new Date(entry.timestamp)) : JulianDate.fromDate(new Date(currentTimestamp))

                if (!this.entities.has(nid)) {
                    if (robot.point) {
                        this._createEntity(robot, useTime)
                    }
                } else {
                    this._updateEntity(robot, useTime)
                }
            }
        },

        async drawFlightPath(nid: string, aroundTime?: number, entryCount: number = 50) {
            if (!this.viewer) return
            const historyStore = useDroneHistoryStore()

            // Ensure data is loaded into cache first
            const currentTime = aroundTime ?? useTimeStore().currentTime
            const windowMs = entryCount * 1000
            await historyStore.ensureDroneDataLoaded(nid, currentTime, windowMs)

            const history = historyStore.cache.get(nid)
            if (!history || history.length === 0) return

            // Define window
            const startTs = currentTime - (windowMs / 2)
            const endTs = currentTime + (windowMs / 2)

            // Optimized slice using sorted indices
            let startIdx = 0, endIdx = history.length - 1
            while (startIdx < history.length && history[startIdx].timestamp < startTs) startIdx++
            while (endIdx >= 0 && history[endIdx].timestamp > endTs) endIdx--

            const entries = history.slice(startIdx, endIdx + 1)
            if (entries.length < 2) return

            // Clean up previous path
            this.removeFlightPathLines(nid)

            const positions = entries.map(e => {
                const p = e.data.point
                return p ? Cartesian3.fromDegrees(p.lon, p.lat, p.alt ?? 150) : null
            }).filter((p): p is Cartesian3 => p !== null)

            // Draw Segments with lerped colors
            for (let i = 0; i < positions.length - 1; i++) {
                const p1 = positions[i]
                const p2 = positions[i + 1]
                const dist = Cartesian3.distance(p1, p2)
                const isDotted = dist > 20 // threshold for "missing data" or jumps

                const progress = i / (positions.length - 1)
                const color = Color.lerp(
                    Color.fromCssColorString("#001144"),
                    Color.fromCssColorString("#00aaff"),
                    progress,
                    new Color()
                )

                this.viewer.entities.add({
                    id: `${nid}-flightPath-${entries[i].timestamp}`,
                    polyline: {
                        positions: [p1, p2],
                        width: isDotted ? 2 : 4,
                        material: isDotted
                            ? new PolylineDashMaterialProperty({ color, dashPattern: 255 })
                            : new PolylineOutlineMaterialProperty({
                                color,
                                outlineColor: Color.BLACK.withAlpha(0.5),
                                outlineWidth: 1
                            })
                    }
                })
            }
        },

        removeFlightPathLines(nid: string) {
            if (!this.viewer) return
            const prefix = `${nid}-flightPath-`
            const toRemove = this.viewer.entities.values.filter(e =>
                typeof e.id === 'string' && e.id.startsWith(prefix)
            )
            toRemove.forEach(e => this.viewer?.entities.remove(e))
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
                model: {
                    uri: robot.type === 'nuc' ? nucModel : droneModel,
                    scale: 1.2
                },
                label: {
                    text: robot.display_name ?? 'Drone',
                    font: '14px monospace',
                    style: LabelStyle.FILL_AND_OUTLINE,
                    outlineWidth: 3,
                    verticalOrigin: VerticalOrigin.BOTTOM,
                    pixelOffset: new Cartesian2(0, -25),
                    disableDepthTestDistance: Number.POSITIVE_INFINITY
                },
                point: {
                    pixelSize: 12,
                    color: new CallbackProperty(() => getDroneColor(robot), false),
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

            // Ground Connection Line
            const lineEnt = this.viewer.entities.add({
                id: `${robot.nid}-line`,
                polyline: {
                    positions: new CallbackProperty(() => {
                        const now = JulianDate.fromDate(new Date(useTimeStore().currentTime))
                        const dronePos = sampled.getValue(now)
                        if (!dronePos) return []
                        const carto = Ellipsoid.WGS84.cartesianToCartographic(dronePos)
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

            // Update the underlying robot data for the color callback
            this.robots.set(robot.nid, robot)
        },

        clear() {
            if (this.viewer) {
                this.entities.forEach(ent => this.viewer!.entities.remove(ent))
                this.connectionLines.forEach(line => this.viewer!.entities.remove(line))
            }
            this.entities.clear()
            this.robots.clear()
            this.connectionLines.clear()
            this._flightPathCache.clear()
        }
    }
})