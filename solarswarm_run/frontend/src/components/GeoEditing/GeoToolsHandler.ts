// src/logic/GeoToolsHandler.ts
import * as Cesium from "cesium"
import { SelectedTool, useGeoToolsStore } from "../../stores/GeoToolsStore"
import { FormType } from "../../models/GeoForm"
import { generateUUID } from "three/src/math/MathUtils"
import { toCartographic } from "vesium"

export function initGeoToolsHandler(viewer: Cesium.Viewer) {
    const geoStore = useGeoToolsStore()
    const handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas)
    const scene = viewer.scene

    // --- State ---
    let activePoints: Cesium.Cartesian3[] = []
    let mousePosition = new Cesium.Cartesian3()
    const SNAP_DISTANCE = 15 // Meters

    // --- Hold Logic State ---
    let isHolding = false
    let holdProgress = 0
    let holdStartTime = 0
    const HOLD_DURATION_MS = 1200

    // --- 1. Guide Point & Progress Indicator ---
    const guidePoint = viewer.entities.add({
        position: new Cesium.CallbackPositionProperty(() => {
            if (geoStore.activeTool === SelectedTool.Polygon && activePoints.length > 2) {
                if (Cesium.Cartesian3.distance(activePoints[0], mousePosition) < SNAP_DISTANCE) {
                    return activePoints[0];
                }
            }
            return mousePosition;
        }, false),
        point: {
            pixelSize: 10,
            color: new Cesium.CallbackProperty(() => isHolding && geoStore.activeTool === SelectedTool.Line ? Cesium.Color.fromHsl(0.6, 1, holdProgress) : Cesium.Color.YELLOW, false),
            outlineColor: Cesium.Color.BLACK,
            outlineWidth: 2,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
        label: {
            text: new Cesium.CallbackProperty(() => {
                const tool = geoStore.activeTool;
                if (isHolding && tool === SelectedTool.Line) return `Finishing... ${Math.round(holdProgress * 100)}%`;
                if (tool === SelectedTool.Polygon && activePoints.length > 2) {
                    if (Cesium.Cartesian3.distance(activePoints[0], mousePosition) < SNAP_DISTANCE) return "CLOSE POLYGON";
                }
                if (tool === SelectedTool.Marker) {
                    const carto = Cesium.Cartographic.fromCartesian(mousePosition);
                    if (!carto) return "Unknown Location";

                    const lat = Cesium.Math.toDegrees(carto.latitude).toFixed(6);
                    const lon = Cesium.Math.toDegrees(carto.longitude).toFixed(6);
                    const alt = carto.height.toFixed(1);

                    return `Lat: ${lat}°, Lon: ${lon}°, Alt: ${alt}m`;
                }
                return "";
            }, false),
            font: '12px monospace',
            showBackground: true,
            backgroundColor: Cesium.Color.BLACK.withAlpha(1.0),
            heightReference: (geoStore.activeTool === SelectedTool.Marker) ? Cesium.HeightReference.NONE : Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.MAX_VALUE,
            pixelOffset: new Cesium.Cartesian2(0, -5),
            verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
        },
        show: false,
    })

    // --- 2. PERFORMANCE PREVIEW ENTITY ---
    const tempEntity = viewer.entities.add({
        position: new Cesium.CallbackPositionProperty(() => activePoints.length > 0 ? activePoints[0] : mousePosition, false),
        polygon: {
            hierarchy: new Cesium.CallbackProperty(() => {
                const tool = geoStore.activeTool
                if (activePoints.length === 0 || ![SelectedTool.Polygon, SelectedTool.Plane].includes(tool)) return null
                if (tool === SelectedTool.Plane) return new Cesium.PolygonHierarchy(computePlaneCorners(activePoints[0], mousePosition).corners)
                return new Cesium.PolygonHierarchy([...activePoints, mousePosition])
            }, false),
            material: Cesium.Color.YELLOW.withAlpha(0.3),
            outline: true,
            outlineColor: Cesium.Color.BLACK,
            outlineWidth: 2,
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND
        },
        ellipse: {
            semiMinorAxis: new Cesium.CallbackProperty(() => (geoStore.activeTool === SelectedTool.Circle && activePoints.length > 0) ? Math.max(Cesium.Cartesian3.distance(activePoints[0], mousePosition), 0.1) : undefined, false),
            semiMajorAxis: new Cesium.CallbackProperty(() => (geoStore.activeTool === SelectedTool.Circle && activePoints.length > 0) ? Math.max(Cesium.Cartesian3.distance(activePoints[0], mousePosition), 0.1) : undefined, false),
            material: Cesium.Color.BLUE.withAlpha(0.4),
            outline: true,
            outlineColor: Cesium.Color.BLACK,
        },
        ellipsoid: {
            radii: new Cesium.CallbackProperty(() => {
                if (geoStore.activeTool !== SelectedTool.Sphere || activePoints.length === 0) return undefined
                const r = Math.max(Cesium.Cartesian3.distance(activePoints[0], mousePosition), 0.1)
                return new Cesium.Cartesian3(r, r, r)
            }, false),
            material: Cesium.Color.RED.withAlpha(0.4),
            outline: true,
            outlineColor: Cesium.Color.BLACK,
        },
        polyline: {
            positions: new Cesium.CallbackProperty(() => {
                const tool = geoStore.activeTool
                if (activePoints.length === 0 || ![SelectedTool.Line, SelectedTool.Circle, SelectedTool.Sphere].includes(tool)) return null
                return (tool === SelectedTool.Line) ? [...activePoints, mousePosition] : [activePoints[0], mousePosition]
            }, false),
            width: 3,
            material: Cesium.Color.YELLOW,
        },
    })

    // --- 3. HELPERS ---
    function computePlaneCorners(center: Cesium.Cartesian3, target: Cesium.Cartesian3) {
        const normal = Cesium.Ellipsoid.WGS84.geodeticSurfaceNormal(center, new Cesium.Cartesian3())
        const cameraUp = viewer.camera.upWC
        const dot = Cesium.Cartesian3.dot(cameraUp, normal)
        const projectedUp = Cesium.Cartesian3.subtract(cameraUp, Cesium.Cartesian3.multiplyByScalar(normal, dot, new Cesium.Cartesian3()), new Cesium.Cartesian3())
        Cesium.Cartesian3.normalize(projectedUp, projectedUp)
        const xAxis = Cesium.Cartesian3.cross(projectedUp, normal, new Cesium.Cartesian3())
        Cesium.Cartesian3.normalize(xAxis, xAxis)
        const offset = Cesium.Cartesian3.subtract(target, center, new Cesium.Cartesian3())
        const w = Cesium.Cartesian3.dot(offset, xAxis); const h = Cesium.Cartesian3.dot(offset, projectedUp)
        return {
            corners: [
                center,
                Cesium.Cartesian3.add(center, Cesium.Cartesian3.multiplyByScalar(xAxis, w, new Cesium.Cartesian3()), new Cesium.Cartesian3()),
                Cesium.Cartesian3.add(center, Cesium.Cartesian3.add(Cesium.Cartesian3.multiplyByScalar(xAxis, w, new Cesium.Cartesian3()), Cesium.Cartesian3.multiplyByScalar(projectedUp, h, new Cesium.Cartesian3()), new Cesium.Cartesian3()), new Cesium.Cartesian3()),
                Cesium.Cartesian3.add(center, Cesium.Cartesian3.multiplyByScalar(projectedUp, h, new Cesium.Cartesian3()), new Cesium.Cartesian3()),
            ], w: Math.abs(w), h: Math.abs(h)
        }
    }

    const addLabel = (pos: Cesium.Cartesian3, text: string, toGround = true) => {
        return viewer.entities.add({
            position: pos,
            label: {
                text: text,
                font: '14px monospace',
                showBackground: true,
                backgroundColor: Cesium.Color.BLACK.withAlpha(1.0),
                heightReference: toGround ? Cesium.HeightReference.CLAMP_TO_GROUND : Cesium.HeightReference.NONE,
                disableDepthTestDistance: Number.MAX_VALUE,
                pixelOffset: new Cesium.Cartesian2(0, -5),
                verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
            }
        });
    }

    const getSurfaceMidpoint = (p1: Cesium.Cartesian3, p2: Cesium.Cartesian3) => {
        const c1 = Cesium.Cartographic.fromCartesian(p1);
        const c2 = Cesium.Cartographic.fromCartesian(p2);
        return Cesium.Cartesian3.fromDegrees(
            (Cesium.Math.toDegrees(c1.longitude) + Cesium.Math.toDegrees(c2.longitude)) / 2,
            (Cesium.Math.toDegrees(c1.latitude) + Cesium.Math.toDegrees(c2.latitude)) / 2,
            0 // CLAMP_TO_GROUND will handle the vertical position
        );
    };

    const finalizeShape = () => {
        const tool = geoStore.activeTool;

        if (activePoints.length === 0 && tool !== SelectedTool.Marker) return;

        const id = generateUUID();
        const labels: Cesium.Entity[] = [];
        let primary: Cesium.Entity;
        let type: FormType = FormType.Polygon;
        let radius = 0;

        let finalPoints = [...activePoints];
        const finalMouse = Cesium.Cartesian3.clone(mousePosition);

        if (tool === SelectedTool.Marker) {
            type = FormType.Marker;
            finalPoints = [finalMouse]
            primary = viewer.entities.add({
                position: finalMouse,
                point: {
                    pixelSize: 12,
                    color: Cesium.Color.RED,
                    outlineColor: Cesium.Color.WHITE,
                    outlineWidth: 3,
                    disableDepthTestDistance: Number.POSITIVE_INFINITY
                }
            });
        }

        else if (tool === SelectedTool.Plane) {
            type = FormType.Plane;
            const { corners, w, h } = computePlaneCorners(finalPoints[0], finalMouse);
            finalPoints = corners;
            primary = viewer.entities.add({ polygon: { hierarchy: new Cesium.PolygonHierarchy(corners), material: Cesium.Color.GREEN.withAlpha(0.5), outline: true, outlineColor: Cesium.Color.BLACK, heightReference: Cesium.HeightReference.CLAMP_TO_GROUND } });
            labels.push(addLabel(getSurfaceMidpoint(corners[0], corners[1]), `W: ${w.toFixed(1)}m`));
            labels.push(addLabel(getSurfaceMidpoint(corners[1], corners[2]), `H: ${h.toFixed(1)}m`));
            labels.push(addLabel(Cesium.BoundingSphere.fromPoints(corners).center, `Area: ${(w * h).toFixed(1)}m²`));

        } else if (tool === SelectedTool.Line) {
            type = FormType.LineString;
            primary = viewer.entities.add({ polyline: { positions: finalPoints, width: 4, material: Cesium.Color.CYAN, clampToGround: true } });
            let total = 0;
            for (let i = 0; i < finalPoints.length - 1; i++) {
                const d = Cesium.Cartesian3.distance(finalPoints[i], finalPoints[i + 1]);
                total += d;
                labels.push(addLabel(getSurfaceMidpoint(finalPoints[i], finalPoints[i + 1]), `${d.toFixed(1)}m`));
            }
            labels.push(addLabel(finalPoints[finalPoints.length - 1], `Total: ${total.toFixed(1)}m`));

        } else if (tool === SelectedTool.Polygon) {
            type = FormType.Polygon;
            primary = viewer.entities.add({ polygon: { hierarchy: new Cesium.PolygonHierarchy(finalPoints), material: Cesium.Color.ORANGE.withAlpha(0.5), outline: true, outlineColor: Cesium.Color.BLACK, heightReference: Cesium.HeightReference.CLAMP_TO_GROUND } });
            for (let i = 0; i < finalPoints.length; i++) {
                const next = finalPoints[(i + 1) % finalPoints.length];
                labels.push(addLabel(getSurfaceMidpoint(finalPoints[i], next), `${Cesium.Cartesian3.distance(finalPoints[i], next).toFixed(1)}m`));
            }

        } else if (tool === SelectedTool.Circle || tool === SelectedTool.Sphere) {
            type = tool === SelectedTool.Sphere ? FormType.Sphere : FormType.Circle;
            radius = Math.max(Cesium.Cartesian3.distance(finalPoints[0], finalMouse), 0.1);
            const graphics = tool === SelectedTool.Sphere ?
                { ellipsoid: { radii: new Cesium.Cartesian3(radius, radius, radius), material: Cesium.Color.RED.withAlpha(0.5), outline: true, outlineColor: Cesium.Color.BLACK } } :
                { ellipse: { semiMinorAxis: radius, semiMajorAxis: radius, material: Cesium.Color.BLUE.withAlpha(0.5), outline: true, outlineColor: Cesium.Color.BLACK, heightReference: Cesium.HeightReference.CLAMP_TO_GROUND } };
            primary = viewer.entities.add({ position: finalPoints[0], ...graphics });
            labels.push(addLabel(finalPoints[0], `Radius: ${radius.toFixed(1)}m`));
        }

        const geoPoints = finalPoints.map(p => {
            const c = Cesium.Cartographic.fromCartesian(p);
            return { lon: Cesium.Math.toDegrees(c.longitude), lat: Cesium.Math.toDegrees(c.latitude), height: c.height };
        });

        geoStore.addForm({ id, name: `${type} ${geoStore.forms.length + 1}`, type, data: { points: geoPoints, radius } }, { primary, labels });
        activePoints = [];
        geoStore.setTool(SelectedTool.None);
    };

    // --- Interaction ---
    handler.setInputAction((movement: any) => {
        if (geoStore.activeTool === SelectedTool.None) { guidePoint.show = false; return; }
        let cartesian: Cesium.Cartesian3 | undefined;

        if (scene.pickPositionSupported) {
            cartesian = scene.pickPosition(movement.endPosition);
        }

        if (!cartesian) {
            const ray = viewer.camera.getPickRay(movement.endPosition);
            if (ray) {
                cartesian = scene.globe.pick(ray, scene);
            }
        }

        if (cartesian) {
            mousePosition = cartesian;
            guidePoint.show = true;
        }

    }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);

    handler.setInputAction(() => {
        if (geoStore.activeTool === SelectedTool.None) return;
        isHolding = true; holdProgress = 0; holdStartTime = performance.now();
        const step = () => {
            if (!isHolding) return;
            const elapsed = performance.now() - holdStartTime; holdProgress = Cesium.Math.clamp(elapsed / HOLD_DURATION_MS, 0, 1);
            if (holdProgress >= 1 && geoStore.activeTool === SelectedTool.Line) { isHolding = false; finalizeShape(); }
            else { requestAnimationFrame(step); }
        }; requestAnimationFrame(step);
    }, Cesium.ScreenSpaceEventType.RIGHT_DOWN);

    handler.setInputAction(() => {
        if (!isHolding) return;
        const tool = geoStore.activeTool; const elapsed = performance.now() - holdStartTime;
        if (tool === SelectedTool.Marker) {
            finalizeShape();
        }
        else if (elapsed < (tool === SelectedTool.Line ? HOLD_DURATION_MS : 250)) {
            if (tool === SelectedTool.Polygon && activePoints.length > 2 && Cesium.Cartesian3.distance(activePoints[0], mousePosition) < SNAP_DISTANCE) { finalizeShape(); }
            else {
                activePoints.push(Cesium.Cartesian3.clone(mousePosition));
                if ([SelectedTool.Plane, SelectedTool.Circle, SelectedTool.Sphere].includes(tool) && activePoints.length === 2) finalizeShape();
            }
        }
        isHolding = false; holdProgress = 0;
    }, Cesium.ScreenSpaceEventType.RIGHT_UP);

    handler.setInputAction(() => { if ([SelectedTool.Polygon, SelectedTool.Line].includes(geoStore.activeTool)) finalizeShape() }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);

    return handler;
}

export async function loadSavedGeoTools(viewer: Cesium.Viewer) {
    const geoStore = useGeoToolsStore();
    const forms = await geoStore.loadFromDisk();

    // Internal helper to ensure reloaded labels use the same clamping logic
    const addL = (pos: Cesium.Cartesian3, text: string) => {
        return viewer.entities.add({
            position: pos,
            label: {
                text,
                font: '12px monospace',
                showBackground: true,
                backgroundColor: Cesium.Color.BLACK.withAlpha(0.8),
                heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
                disableDepthTestDistance: Number.MAX_VALUE
            }
        });
    };

    const getMid = (p1: Cesium.Cartesian3, p2: Cesium.Cartesian3) => {
        const c1 = Cesium.Cartographic.fromCartesian(p1);
        const c2 = Cesium.Cartographic.fromCartesian(p2);
        return Cesium.Cartesian3.fromDegrees(
            Cesium.Math.toDegrees((c1.longitude + c2.longitude) / 2),
            Cesium.Math.toDegrees((c1.latitude + c2.latitude) / 2),
            0
        );
    };

    forms.forEach(form => {
        const points = form.data.points.map((p: any) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat, p.height));
        let primary: Cesium.Entity | null = null;
        const labels: Cesium.Entity[] = [];

        if (form.type === FormType.Marker) {
            primary = viewer.entities.add({
                position: points[0],
                point: {
                    pixelSize: 12,
                    color: Cesium.Color.RED,
                    outlineColor: Cesium.Color.WHITE,
                    outlineWidth: 3,
                    heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
                    disableDepthTestDistance: Number.POSITIVE_INFINITY
                }
            });
            const carto = Cesium.Cartographic.fromCartesian(points[0]);
            if (carto) {
                const txt = `Lat: ${Cesium.Math.toDegrees(carto.latitude).toFixed(6)}°, Lon: ${Cesium.Math.toDegrees(carto.longitude).toFixed(6)}°, Alt: ${carto.height.toFixed(1)}m`;
                labels.push(addL(points[0], txt));
            }
        }

        if (form.type === FormType.LineString) {
            primary = viewer.entities.add({ polyline: { positions: points, width: 4, material: Cesium.Color.CYAN, clampToGround: true } });
            let total = 0;
            for (let i = 0; i < points.length - 1; i++) {
                const d = Cesium.Cartesian3.distance(points[i], points[i + 1]);
                total += d;
                labels.push(addL(getMid(points[i], points[i + 1]), `${d.toFixed(1)}m`));
            }
            labels.push(addL(points[points.length - 1], `Total: ${total.toFixed(1)}m`));

        } else if (form.type === FormType.Polygon || form.type === FormType.Plane) {
            const color = form.type === FormType.Plane ? Cesium.Color.GREEN : Cesium.Color.ORANGE;
            primary = viewer.entities.add({
                polygon: {
                    hierarchy: new Cesium.PolygonHierarchy(points),
                    material: color.withAlpha(0.5),
                    outline: true,
                    outlineColor: Cesium.Color.BLACK,
                    heightReference: Cesium.HeightReference.CLAMP_TO_GROUND
                }
            });
            for (let i = 0; i < points.length; i++) {
                labels.push(addL(getMid(points[i], points[(i + 1) % points.length]), `${Cesium.Cartesian3.distance(points[i], points[(i + 1) % points.length]).toFixed(1)}m`));
            }
        } else if (form.type === FormType.Circle || form.type === FormType.Sphere) {
            const r = form.data.radius || 10;
            const graphics = (form.type === FormType.Sphere)
                ? { ellipsoid: { radii: new Cesium.Cartesian3(r, r, r), material: Cesium.Color.RED.withAlpha(0.5), outline: true, outlineColor: Cesium.Color.BLACK } }
                : { ellipse: { semiMinorAxis: r, semiMajorAxis: r, material: Cesium.Color.BLUE.withAlpha(0.5), outline: true, outlineColor: Cesium.Color.BLACK, heightReference: Cesium.HeightReference.CLAMP_TO_GROUND } };
            primary = viewer.entities.add({ position: points[0], ...graphics });
            labels.push(addL(points[0], `R: ${r.toFixed(1)}m`));
        }
        if (primary) geoStore.entityList.set(form.id, { primary, labels });
    });
}