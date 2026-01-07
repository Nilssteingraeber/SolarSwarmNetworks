// We can ignore this as vue correctly loads this data
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
    ClockRange,
    Viewer,
    ExtrapolationType,
    PolylineDashMaterialProperty
} from 'cesium'

import { useDroneHistoryStore, RobotHistoryEntry } from './DroneHistoryStore'
import { useTimeStore } from '../stores/TimeStore'

import type { Robot, RobotWithEntity } from '../models/Robot'
import { Hash } from 'crypto'

const getDroneColor = (robot: Robot) => {
    const now = useTimeStore().currentTime
    const lastHeard = robot.last_heard || 0  // assume this is always up-to-date
    const timeSinceLast = (now - lastHeard) * 1000  // seconds

    if (timeSinceLast < 5) return Color.GREEN
    if (timeSinceLast < 7) return Color.YELLOW
    if (timeSinceLast < 9) return Color.RED
    return Color.GRAY
}


export const useDroneEntityStore = defineStore('droneEntities', {
    state: () => ({
        viewer: null as Viewer | null,

        // nid -> Cesium Entity
        entities: new Map<string, Entity>(),

        // nid -> Robot data
        robots: new Map<string, Robot>(),

        // connection id -> polyline entity
        connectionLines: new Map<string, Entity>(),

        _flightPathCache: new Map<string, { start: number; end: number }[]>(),

        _tickHandler: undefined as ((clock: any) => void) | undefined
    }),

    getters: {
        /** Returns array of { robot, entity } */
        getRobots(state): RobotWithEntity[] {
            const list: RobotWithEntity[] = []
            state.robots.forEach((robot, nid) => {
                const ent = state.entities.get(nid)
                if (ent) list.push({ robot, entity: ent })
            })
            return list
        },

        /** Returns single { robot, entity } */
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
            const currentTime = JulianDate.fromDate(new Date(timeStore.currentTime))
            const currentTimestamp = JulianDate.toDate(currentTime).getTime()

            // Get drones active in time window
            const dronesMap = await historyStore.getDronesInWindow(currentTimestamp, 2500)
            const seen = new Set<string>()

            for (const [nid, robot] of dronesMap.entries()) {
                seen.add(nid)
                this.robots.set(nid, robot)

                // Get the actual history entry with recording timestamp
                const historyEntry = await historyStore.getDroneEntryAtTime(nid, currentTimestamp)

                if (!this.entities.has(nid)) {
                    if (historyEntry?.data.point || robot.point) {
                        const useTime = historyEntry?.timestamp || JulianDate.toDate(currentTime).getTime()
                        this._createEntity(robot, JulianDate.fromDate(new Date(useTime)))
                    }
                    continue
                }

                // Update using history entry's actual recording time
                if (historyEntry) {
                    this._updateEntity(historyEntry.data, JulianDate.fromDate(new Date(historyEntry.timestamp)))
                } else {
                    this._updateEntity(robot, currentTime)
                }
            }
        },



        async drawFlightPath(nid: string, aroundTime?: number, entryCount: number = 50) {
            if (!this.viewer) return
            const historyStore = useDroneHistoryStore()
            const now = useTimeStore().currentTime

            let desiredStart: number
            let desiredEnd: number

            if (aroundTime !== undefined) {
                const halfWindow = entryCount * 1000 // adjust timestep as needed
                desiredStart = aroundTime - halfWindow
                desiredEnd = aroundTime + halfWindow
            } else {
                desiredStart = 0
                desiredEnd = Number.MAX_SAFE_INTEGER
            }

            // --- Determine missing ranges to fetch ---
            const drawnRanges = this._flightPathCache.get(nid) ?? []
            const missingRanges: { start: number; end: number }[] = []

            let currentStart = desiredStart
            for (const range of drawnRanges) {
                if (range.end < currentStart) continue
                if (range.start > desiredEnd) break

                if (range.start > currentStart) {
                    missingRanges.push({ start: currentStart, end: range.start - 1 })
                }
                currentStart = Math.max(currentStart, range.end + 1)
            }
            if (currentStart <= desiredEnd) missingRanges.push({ start: currentStart, end: desiredEnd })

            // --- Fetch missing entries ---
            let entries: RobotHistoryEntry[] = []
            if (missingRanges.length) {
                for (const r of missingRanges) {
                    const fetched = await historyStore.loadIncrementalHistory(nid, r.start, r.end)
                    const nidMap = historyStore.cache.get(nid)
                    if (nidMap) {
                        entries.push(...Array.from(nidMap.values()).filter(e => e.timestamp >= r.start && e.timestamp <= r.end))
                    }
                }
            }

            // --- Include already drawn entries from cache ---
            for (const range of drawnRanges) {
                const nidMap = historyStore.cache.get(nid)
                if (!nidMap) continue
                entries.push(...Array.from(nidMap.values()).filter(e => e.timestamp >= range.start && e.timestamp <= range.end))
            }

            if (!entries.length) return

            // --- Sort entries and create positions ---
            entries.sort((a, b) => a.timestamp - b.timestamp)
            const positions = entries.map(e => {
                const point = e.data.point
                if (!point) return null
                return Cartesian3.fromDegrees(point.lon, point.lat, point.alt ?? 150)
            }).filter((p): p is Cartesian3 => p !== null)
            if (!positions.length) return

            // --- Remove previous flight path if full redraw ---
            if (aroundTime === undefined) {
                this.removeFlightPathLines(nid)
                this._flightPathCache.set(nid, [])
            }

            // --- Draw segments ---
            const segments: { positions: Cartesian3[]; dotted: boolean; distancePassed: number }[] = []
            let distanceTotal = 0
            const distance = (p1: Cartesian3, p2: Cartesian3) => Cartesian3.distance(p1, p2)

            for (let i = 0; i < positions.length - 1; i++) {
                const p1 = positions[i]
                const p2 = positions[i + 1]
                const segDist = distance(p1, p2)
                distanceTotal += segDist
                segments.push({ positions: [p1, p2], dotted: segDist > 15, distancePassed: distanceTotal })
            }

            const maxDistance = distanceTotal
            const getColor = (distancePassed: number) => {
                const t = Math.min(distancePassed / maxDistance, 1)
                return Color.lerp(Color.fromCssColorString("#001144"), Color.fromCssColorString("#00aaff"), t, new Color())
            }

            for (let i = 0; i < segments.length; i++) {
                const seg = segments[i]
                const startTs = entries[i].timestamp
                const endTs = entries[i + 1].timestamp
                this.viewer.entities.add({
                    id: `${nid}-flightPath-${startTs}-${endTs}`,
                    polyline: {
                        positions: seg.positions,
                        width: seg.dotted ? 2 : 6,
                        material: seg.dotted
                            ? new PolylineDashMaterialProperty({ color: getColor(seg.distancePassed), dashPattern: 255 })
                            : new PolylineOutlineMaterialProperty({
                                color: getColor(seg.distancePassed),
                                outlineColor: Color.BLACK.withAlpha(0.3),
                                outlineWidth: 2
                            })
                    }
                })
            }

            // --- Update cached ranges ---
            this._flightPathCache.set(nid, [...drawnRanges, { start: desiredStart, end: desiredEnd }])
        },

        removeFlightPathLines(nid: string) {
            if (!this.viewer) return
            const prefix = `${nid}-flightPath-`

            const toRemove: Entity[] = []
            this.viewer.entities.values.forEach(entity => {
                if (entity.id && typeof entity.id === 'string' && entity.id.startsWith(prefix)) toRemove.push(entity)
            })
            toRemove.forEach(entity => this.viewer?.entities.remove(entity))

            // Reset flight path cache for this drone
            this._flightPathCache.delete(nid)
        }
        ,

        _createEntity(robot: Robot, currentTime: JulianDate) {
            if (!this.viewer || !robot.point) return

            const { lon, lat, alt } = robot.point
            console.debug(robot.point)
            const pos = Cartesian3.fromDegrees(lon, lat, alt ?? 150)



            const sampled = new SampledPositionProperty()
            sampled.forwardExtrapolationType = ExtrapolationType.HOLD
            sampled.backwardExtrapolationType = ExtrapolationType.HOLD
            sampled.addSample(currentTime, pos)

            const ent = this.viewer.entities.add({
                id: robot.nid,
                position: sampled,
                orientation: new VelocityOrientationProperty(sampled),
                model: {
                    uri: Math.random() > 0.5 ? droneModel : nucModel,
                    scale: 1.2
                },
                label: {
                    text: robot.display_name ?? 'Drone',
                    font: '16px sans-serif',
                    style: LabelStyle.FILL_AND_OUTLINE,
                    outlineWidth: 4,
                    verticalOrigin: VerticalOrigin.BOTTOM,
                    fillColor: Color.WHITE,
                    outlineColor: Color.BLACK,
                    disableDepthTestDistance: Number.POSITIVE_INFINITY,
                    pixelOffset: new Cartesian2(0, -20)
                },
                point: {
                    pixelSize: 16,
                    color: new CallbackProperty(() => getDroneColor(robot), false),
                    outlineColor: Color.BLACK,
                    outlineWidth: 2,
                    show: true,
                    disableDepthTestDistance: Number.POSITIVE_INFINITY
                },
                properties: {
                    isSelectable: new ConstantProperty(true),
                    droneData: new ConstantProperty(robot)
                }
            })

            this.entities.set(robot.nid, markRaw(ent))

            // Add vertical line below drone with same timestamp from timeStore
            const lineEnt = this.viewer.entities.add({
                id: `${robot.nid}-line`,
                polyline: {
                    positions: new CallbackProperty(() => {
                        const timeStore = useTimeStore()
                        const now = JulianDate.fromDate(new Date(timeStore.currentTime))
                        const dronePos = sampled.getValue(now)
                        if (!dronePos) return []

                        const carto = Ellipsoid.WGS84.cartesianToCartographic(dronePos)
                        const downPos = Ellipsoid.WGS84.cartographicToCartesian(
                            new Cartographic(carto.longitude, carto.latitude, 0)
                        )
                        return [dronePos, downPos]
                    }, false),
                    width: 5,
                    material: new PolylineOutlineMaterialProperty({
                        color: Color.fromCssColorString("#e5d84b"),
                        outlineColor: Color.BLACK,
                        outlineWidth: 2,
                    }),
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
                // Use the actual recording time, not simulation time + offset
                ent.position.addSample(recordedTime, newPos)
            }

            ent.point!.color = new CallbackProperty(() => getDroneColor(robot), false)
            ent.properties!.droneData = new ConstantProperty(robot)
        },



        clear() {
            if (this.viewer) {
                for (const ent of this.entities.values())
                    this.viewer.entities.remove(ent)
                for (const line of this.connectionLines.values())
                    this.viewer.entities.remove(line)

                if (this._tickHandler)
                    this.viewer.clock.onTick.removeEventListener(this._tickHandler)
            }

            this.entities.clear()
            this.robots.clear()
            this.connectionLines.clear()
        }
    }
})
