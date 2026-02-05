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
    GeometryInstance,
    PolylineGeometry,
    PolylineColorAppearance,
    PointPrimitiveCollection,
    Primitive,
    LinearApproximation,
    ArcType
} from 'cesium'

import { useDroneHistoryStore, initDBRead } from './DroneHistoryStore'
import { useTimeStore } from '../stores/TimeStore'
import type { Robot, RobotWithEntity } from '../models/Robot'


const getDroneColor = (nid: string) => {
    const historyStore = useDroneHistoryStore();
    const timeStore = useTimeStore();

    // 1. Access the history cache for this drone
    const history = historyStore.cache.get(nid);
    if (!history || history.length === 0) return Color.GRAY;

    const currentTime = timeStore.currentTime;

    // 2. Use our existing binary search to find the index closest to the playhead
    // We assume findClosestIndex is exported or accessible here
    const idx = historyStore.findClosestIndex(history, currentTime);
    const closestEntry = history[idx];

    // 3. Extract the timestamp from the actual history entry
    const heardMs = closestEntry.data.last_heard ?? 0;

    // 4. Unit-agnostic comparison
    const nowMs = currentTime;
    const actualHeardMs = heardMs;

    // We take the ABSOLUTE difference. 
    // If the playhead is at 10:00:00 and the closest stamp is 10:00:05, 
    // it's still "5 second old" data in terms of accuracy.
    const lagSeconds = Math.abs(nowMs - actualHeardMs) / 1000.0;

    // Throttled Debug (Per drone, roughly every 2 seconds)
    // if (Math.floor(nowMs) % 2000 < 20) {
    //     console.log(`[Replay Status] ${nid} | Playhead: ${new Date(nowMs).toLocaleTimeString()} | Closest Stamp: ${new Date(actualHeardMs).toLocaleTimeString()} | Gap: ${lagSeconds.toFixed(1)}s`);
    // }

    // 5. Apply the requested thresholds
    if (lagSeconds < 10) return Color.GREEN;
    if (lagSeconds < 15) return Color.YELLOW;
    if (lagSeconds < 25) return Color.RED;

    return Color.GRAY;
}

export const useDroneEntityStore = defineStore('droneEntities', {
    state: () => ({
        viewer: null as Viewer | null,
        entities: markRaw(new Map<string, Entity>()),
        robots: markRaw(new Map<string, Robot>()),
        stateNames: new Map<number, string>(),
        connectionLines: markRaw(new Map<string, Entity>()),
        lastSyncTime: 0,
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
            if (!this.viewer) return;
            const historyStore = useDroneHistoryStore();
            const timeStore = useTimeStore();
            const currentTimestamp = timeStore.currentTime;

            // 1. Detect Jump or Rewind (Discontinuity)
            const timeDelta = currentTimestamp - this.lastSyncTime;
            const isDiscontinuous = Math.abs(timeDelta) > 1500 || timeDelta < 0;
            this.lastSyncTime = currentTimestamp;

            const dronesMap = await historyStore.getDronesInWindow(currentTimestamp, 5000);

            this.viewer.entities.suspendEvents();

            const updatePromises = Array.from(dronesMap.entries()).map(async ([nid, robot]) => {
                this.robots.set(nid, robot);


                const entry = await historyStore.getSnapshotAt(nid, currentTimestamp, 2000);
                const nextEntry = await this.getNextPointAfter(nid, currentTimestamp, 2000);

                const useTime = entry
                    ? JulianDate.fromDate(new Date(entry.timestamp))
                    : JulianDate.fromDate(new Date(currentTimestamp));

                if (!this.entities.has(nid)) {
                    if (robot.point) this._createEntity(robot, useTime);
                } else {
                    // Update current position
                    this._updateEntity(robot, useTime, isDiscontinuous);

                    // Update next position (aiming)
                    if (nextEntry) {
                        const nextTime = JulianDate.fromDate(new Date(nextEntry.timestamp));
                        this._updateEntity(nextEntry.data, nextTime, false);
                    }
                }
            });

            await Promise.all(updatePromises);
            this.viewer.entities.resumeEvents();
        },

        async getNextPointAfter(nid: string, timestamp: number, windowMs: number = 2000) {
            const db = await initDBRead();
            return new Promise<any>((resolve) => {
                const tx = db.transaction('history', 'readonly');
                const store = tx.objectStore('history');
                const index = store.index('nid_timestamp');

                const range = IDBKeyRange.bound([nid, timestamp + 1], [nid, timestamp + windowMs]);
                const request = index.openCursor(range, "next");

                request.onsuccess = (e: any) => {
                    const cursor = e.target.result;
                    resolve(cursor ? cursor.value : null);
                };
                request.onerror = () => resolve(null);
            });
        },

        async drawFlightPath(nid: string, aroundTime?: number) {
            if (!this.viewer) return;
            const historyStore = useDroneHistoryStore();
            const timeStore = useTimeStore();
            const currentTime = aroundTime ?? timeStore.currentTime;

            // 1. Ensure data exists (unchanged)
            // Reducing window to 2 minutes to keep search fast, we only draw 60s anyway
            await historyStore.ensureDroneDataLoaded(nid, currentTime);

            const history = historyStore.cache.get(nid);
            if (!history || history.length === 0) return;

            // 2. SMART SLICING (Fixes "Short Line" and "Performance")
            // Instead of .filter().slice(-64), we find the start/end indices mathematically.

            // Find the index of "Now"
            const endIndex = historyStore.findClosestIndex(history, currentTime);

            // Walk backwards to find "60 seconds ago"
            // (Heuristic optimization: average drone rate is usually consistent, 
            // but a while-loop is safest and fast enough for <1000 points)
            let startIndex = endIndex;
            const startTime = currentTime - 60000; // 60 Seconds Tail

            while (startIndex > 0 && history[startIndex].timestamp > startTime) {
                startIndex--;
            }

            // Extract only the relevant window
            const entries = history.slice(startIndex, endIndex + 1);

            // Need at least 2 points to make a line
            if (entries.length < 2) {
                this.removeFlightPathLines(nid); // Cleanup if empty
                return;
            }

            // 3. Prepare Geometry Arrays
            const positions: Cartesian3[] = [];
            const colors: Color[] = [];
            const dots: Cartesian3[] = [];

            for (let i = 0; i < entries.length; i++) {
                const p = entries[i].data.point;
                if (!p) continue;

                const pos = Cartesian3.fromDegrees(p.lon, p.lat, p.alt ?? 150);
                positions.push(pos);

                // Optional: Only draw dots every 5th point to save FPS
                if (i % 5 === 0) dots.push(pos);

                // Color Calculation (Your original Rainbow Logic)
                // Normalize "age" based on the actual time window (0 to 60s)
                const ageMs = currentTime - entries[i].timestamp;
                const progress = 1.0 - (ageMs / 60000); // 1.0 = Now, 0.0 = 60s ago

                // Clamp progress to 0-1 just in case
                const safeProgress = Math.max(0, Math.min(1, progress));

                const hue = 0.65 * (1.0 - safeProgress); // Blue -> Red
                const lightness = 0.5 * Math.pow(safeProgress, 0.4);
                colors.push(Color.fromHsl(hue, 1.0, lightness, 1.0));
            }

            // Add the "Live" tip (Current interpolated position)
            const entity = this.entities.get(nid);
            if (entity && entity.position) {
                const nowJulian = JulianDate.fromDate(new Date(currentTime));
                const currentPos = entity.position.getValue(nowJulian);
                if (currentPos) {
                    positions.push(currentPos);
                    colors.push(Color.fromHsl(0.0, 1.0, 0.5, 1.0)); // Bright Red Tip
                }
            }

            // 4. DRAWING (The Primitive Swap)
            // We remove the old one and add the new one. 
            // Doing this 60fps is bad. Doing it 10fps is fine.
            this.removeFlightPathLines(nid);

            if (positions.length >= 2) {
                const linePrimitive = this.viewer.scene.primitives.add(new Primitive({
                    geometryInstances: new GeometryInstance({
                        geometry: new PolylineGeometry({
                            positions: positions,
                            width: 4.0,
                            vertexFormat: PolylineColorAppearance.VERTEX_FORMAT,
                            colors: colors,
                            colorsPerVertex: true,
                            arcType: ArcType.GEODESIC // Smooths long lines
                        })
                    }),
                    appearance: new PolylineColorAppearance({ translucent: false }),
                    asynchronous: false // Keep false to prevent flickering on rapid updates
                }));
                this._flightPathPrimitives.set(nid, markRaw(linePrimitive));
            }

            // Draw Dots (Optional: Consider removing if still laggy)
            if (dots.length > 0) {
                const pointCollection = this.viewer.scene.primitives.add(new PointPrimitiveCollection());
                dots.forEach(pt => {
                    pointCollection.add({
                        position: pt,
                        color: Color.WHITE.withAlpha(0.6),
                        pixelSize: 4,
                        disableDepthTestDistance: Number.POSITIVE_INFINITY
                    });
                });
                this._hoverPointCollections.set(nid, markRaw(pointCollection));
            }
        },

        removeFlightPathLines(nid: string) {
            if (!this.viewer) return;
            const linePrim = this._flightPathPrimitives.get(nid);
            if (linePrim) {
                this.viewer.scene.primitives.remove(linePrim);
                this._flightPathPrimitives.delete(nid);
            }
            const pointColl = this._hoverPointCollections.get(nid);
            if (pointColl) {
                this.viewer.scene.primitives.remove(pointColl);
                this._hoverPointCollections.delete(nid);
            }
        },

        _createEntity(robot: Robot, recordedTime: JulianDate) {
            if (!this.viewer || !robot.point) return;

            const { lon, lat, alt } = robot.point;
            const pos = Cartesian3.fromDegrees(lon, lat, alt ?? 150);

            const sampled = new SampledPositionProperty();
            sampled.forwardExtrapolationType = ExtrapolationType.HOLD;
            sampled.backwardExtrapolationType = ExtrapolationType.HOLD;
            sampled.setInterpolationOptions({
                interpolationDegree: 1, // Snappy linear movement
                interpolationAlgorithm: LinearApproximation
            });
            sampled.addSample(recordedTime, pos);

            const ent = this.viewer.entities.add({
                id: robot.nid,
                position: sampled,
                orientation: new VelocityOrientationProperty(sampled),
                model: { uri: nucModel, scale: 1.2 },
                label: {
                    text: this.stateNames.get(robot.state_id ?? -1) ?? ("No State") + "\n" + (robot.display_name ?? robot.nid),
                    font: '14px monospace',
                    style: LabelStyle.FILL_AND_OUTLINE,
                    outlineWidth: 3,
                    verticalOrigin: VerticalOrigin.BOTTOM,
                    pixelOffset: new Cartesian2(0, -25),
                    disableDepthTestDistance: Number.POSITIVE_INFINITY
                },
                point: {
                    pixelSize: 12,
                    color: new CallbackProperty(() => getDroneColor(robot.nid), false),
                    outlineColor: Color.BLACK,
                    outlineWidth: 2,
                    disableDepthTestDistance: Number.POSITIVE_INFINITY
                },
                properties: {
                    isSelectable: new ConstantProperty(true),
                    droneData: new ConstantProperty(robot)
                }
            });
            this.entities.set(robot.nid, markRaw(ent));

            const lineEnt = this.viewer.entities.add({
                id: `${robot.nid}-line`,
                polyline: {
                    positions: new CallbackProperty((time) => {
                        const dronePos = sampled.getValue(time);
                        if (!dronePos) return [];
                        const carto = Cartographic.fromCartesian(dronePos);
                        const groundPos = Cartesian3.fromRadians(carto.longitude, carto.latitude, 0);
                        return [dronePos, groundPos];
                    }, false),
                    width: 2,
                    material: new PolylineDashMaterialProperty({
                        color: Color.YELLOW.withAlpha(0.6),
                        dashLength: 8
                    })
                }
            });
            this.connectionLines.set(`${robot.nid}-line`, markRaw(lineEnt));
        },

        _updateEntity(robot: Robot, recordedTime: JulianDate, isDiscontinuous: boolean) {
            const ent = this.entities.get(robot.nid);
            if (!ent || !robot.point) return;

            if (ent.position instanceof SampledPositionProperty) {
                // Wipe buffers on jump to prevent "warp lines"
                if (isDiscontinuous) {
                    // @ts-ignore
                    ent.position._property._times = [];
                    // @ts-ignore
                    ent.position._property._values = [];
                }

                // @ts-ignore
                const times = ent.position._property._times;
                const lastSampleTime = times && times.length > 0 ? times[times.length - 1] : null;

                if (!lastSampleTime || JulianDate.compare(recordedTime, lastSampleTime) > 0) {
                    const newPos = Cartesian3.fromDegrees(robot.point.lon, robot.point.lat, robot.point.alt ?? 150);
                    ent.position.addSample(recordedTime, newPos);
                }
            }

            this.robots.set(robot.nid, robot);
            if (ent.properties && ent.properties.droneData) {
                ent.properties.droneData.setValue(robot);
            }
        },

        clear() {
            if (this.viewer) {
                this.entities.forEach(ent => this.viewer!.entities.remove(ent));
                this.connectionLines.forEach(line => this.viewer!.entities.remove(line));
                this._flightPathPrimitives.forEach(prim => this.viewer!.scene.primitives.remove(prim));
                this._hoverPointCollections.forEach(coll => this.viewer!.scene.primitives.remove(coll));
            }
            this.entities.clear();
            this.robots.clear();
            this.connectionLines.clear();
            this._flightPathPrimitives.clear();
            this._hoverPointCollections.clear();
        },

        setStateNames(input: Map<number, string> | Record<string, string>) {
            this.stateNames.clear()

            if (input instanceof Map) {
                for (const [k, v] of input) {
                    this.stateNames.set(k, v)
                }
            } else {
                for (const [k, v] of Object.entries(input)) {
                    this.stateNames.set(Number(k), v)
                }
            }
        }
    }
});