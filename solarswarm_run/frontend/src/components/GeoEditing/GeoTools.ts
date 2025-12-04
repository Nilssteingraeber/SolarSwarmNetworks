import * as Cesium from "cesium"
import { useGeoToolsStore } from "../../stores/GeoToolsStore"
import { FormType, GeoForm, GeoLocation } from "../../models/GeoForm"

export function initGeoToolsHandler(viewer: Cesium.Viewer) {
    const geoTools = useGeoToolsStore()
    const handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas)

    // ================================
    // Plane drawing state
    // ================================
    let planeAnchor: Cesium.Cartesian3 | null = null
    let planeCorners: Cesium.Cartesian3[] = []
    let tempPlaneEntity: Cesium.Entity | null = null
    let planeCounter = 0

    const computePlaneAxes = (normal: Cesium.Cartesian3) => {
        let xAxis = Cesium.Cartesian3.cross(Cesium.Cartesian3.UNIT_Z, normal, new Cesium.Cartesian3())
        if (Cesium.Cartesian3.magnitude(xAxis) === 0) {
            xAxis = Cesium.Cartesian3.cross(Cesium.Cartesian3.UNIT_Y, normal, new Cesium.Cartesian3())
        }
        Cesium.Cartesian3.normalize(xAxis, xAxis)
        const yAxis = Cesium.Cartesian3.cross(normal, xAxis, new Cesium.Cartesian3())
        Cesium.Cartesian3.normalize(yAxis, yAxis)
        return { xAxis, yAxis }
    }

    const computePlaneAxesAlignedToCamera = (normal: Cesium.Cartesian3) => {
        // camera's up direction
        const cameraUp = Cesium.Cartesian3.clone(viewer.camera.upWC)

        // project cameraUp onto the plane
        const dot = Cesium.Cartesian3.dot(cameraUp, normal)
        const projected = Cesium.Cartesian3.subtract(cameraUp, Cesium.Cartesian3.multiplyByScalar(normal, dot, new Cesium.Cartesian3()), new Cesium.Cartesian3())
        Cesium.Cartesian3.normalize(projected, projected)

        const yAxis = projected // points roughly “up” on screen
        const xAxis = Cesium.Cartesian3.cross(yAxis, normal, new Cesium.Cartesian3())
        Cesium.Cartesian3.normalize(xAxis, xAxis)

        return { xAxis, yAxis }
    }


    // --- Plane handlers ---
    function handlePlaneRightClick(movement: Cesium.ScreenSpaceEventHandler.PositionedEvent) {
        const scene = viewer.scene
        if (!scene.pickPositionSupported) return
        const picked = scene.pickPosition(movement.position)
        if (!picked) return

        planeAnchor = picked
        planeCorners = []

        if (!tempPlaneEntity) {
            tempPlaneEntity = viewer.entities.add({
                polygon: {
                    hierarchy: new Cesium.CallbackProperty(() => new Cesium.PolygonHierarchy([...planeCorners]), false),
                    material: Cesium.Color.BLUE.withAlpha(0.3),
                    outline: true,
                    outlineColor: Cesium.Color.WHITE,
                    perPositionHeight: true,
                },
            })
        }
    }

    function handlePlaneMouseMove(movement: Cesium.ScreenSpaceEventHandler.MotionEvent) {
        if (!planeAnchor || !tempPlaneEntity) return
        const scene = viewer.scene
        if (!scene.pickPositionSupported) return
        const picked = scene.pickPosition(movement.endPosition)
        if (!picked) return

        const normal = Cesium.Cartesian3.normalize(
            Cesium.Ellipsoid.WGS84.geodeticSurfaceNormal(planeAnchor, new Cesium.Cartesian3()),
            new Cesium.Cartesian3()
        )

        const { xAxis, yAxis } = computePlaneAxesAlignedToCamera(normal)
        const offset = Cesium.Cartesian3.subtract(picked, planeAnchor, new Cesium.Cartesian3())
        const dx = Cesium.Cartesian3.dot(offset, xAxis)
        const dy = Cesium.Cartesian3.dot(offset, yAxis)

        planeCorners.length = 0
        const corner1 = planeAnchor
        const corner2 = Cesium.Cartesian3.add(planeAnchor, Cesium.Cartesian3.multiplyByScalar(xAxis, dx, new Cesium.Cartesian3()), new Cesium.Cartesian3())
        const corner3 = Cesium.Cartesian3.add(corner2, Cesium.Cartesian3.multiplyByScalar(yAxis, dy, new Cesium.Cartesian3()), new Cesium.Cartesian3())
        const corner4 = Cesium.Cartesian3.add(planeAnchor, Cesium.Cartesian3.multiplyByScalar(yAxis, dy, new Cesium.Cartesian3()), new Cesium.Cartesian3())
        planeCorners.push(corner1, corner2, corner3, corner4)
    }

    function handlePlaneLeftClick() {
        if (!planeAnchor || planeCorners.length < 4) return

        planeCounter++
        const id = `plane-${planeCounter}`

        // Normal plane
        const polygonEntity = viewer.entities.add({
            polygon: {
                hierarchy: new Cesium.PolygonHierarchy([...planeCorners]),
                material: Cesium.Color.RED.withAlpha(0.5),
                outline: true,
                outlineColor: Cesium.Color.BLACK,
                perPositionHeight: true,
            },
        })

        const center = Cesium.BoundingSphere.fromPoints(planeCorners).center
        const labelEntity = viewer.entities.add({
            position: center,
            label: {
                text: `Plane ${planeCounter}`,
                font: "16px sans-serif",
                fillColor: Cesium.Color.WHITE,
                outlineColor: Cesium.Color.BLACK,
                outlineWidth: 2,
                verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
                pixelOffset: new Cesium.Cartesian2(0, -10),
            },
        })

        const points: GeoLocation[] = planeCorners.map((c) => {
            const carto = Cesium.Cartographic.fromCartesian(c)
            return {
                lon: Cesium.Math.toDegrees(carto.longitude),
                lat: Cesium.Math.toDegrees(carto.latitude),
                height: carto.height
            }
        })

        const geoForm: GeoForm = {
            id,
            name: `Plane ${planeCounter}`,
            type: FormType.Plane,
            data: { points, shapeForm: undefined },
        }

        geoTools.addForm(geoForm, {
            polygon: polygonEntity,
            label: labelEntity
        })


        if (tempPlaneEntity) {
            viewer.entities.remove(tempPlaneEntity)
            tempPlaneEntity = null
        }
        planeAnchor = null
        planeCorners = []
    }


    // ================================
    // Circle drawing state
    // ================================
    let circleAnchor: Cesium.Cartesian3 | null = null
    let tempCircleEntity: Cesium.Entity | null = null
    let tempRadiusLine: Cesium.Entity | null = null
    let circleCounter = 0
    let currentRadius = 0

    // --- Circle helpers ---
    function makeCirclePolygon(center: Cesium.Cartesian3, normal: Cesium.Cartesian3, radius: number, segments = 64) {
        const { xAxis, yAxis } = computePlaneAxes(normal)
        const positions: Cesium.Cartesian3[] = []
        for (let i = 0; i < segments; i++) {
            const angle = (i / segments) * 2 * Math.PI
            const dir = Cesium.Cartesian3.add(
                Cesium.Cartesian3.multiplyByScalar(xAxis, Math.cos(angle) * radius, new Cesium.Cartesian3()),
                Cesium.Cartesian3.multiplyByScalar(yAxis, Math.sin(angle) * radius, new Cesium.Cartesian3()),
                new Cesium.Cartesian3()
            )
            positions.push(Cesium.Cartesian3.add(center, dir, new Cesium.Cartesian3()))
        }
        return positions
    }

    // --- Circle handlers ---
    function handleCircleRightClick(movement: Cesium.ScreenSpaceEventHandler.PositionedEvent) {
        const scene = viewer.scene
        if (!scene.pickPositionSupported) return
        const picked = scene.pickPosition(movement.position)
        if (!picked) return

        circleAnchor = picked
        currentRadius = 0

        if (!tempCircleEntity) {
            tempCircleEntity = viewer.entities.add({
                polygon: {
                    hierarchy: new Cesium.CallbackProperty(() => new Cesium.PolygonHierarchy([]), false),
                    material: Cesium.Color.BLUE.withAlpha(0.3),
                    outline: true,
                    outlineColor: Cesium.Color.WHITE,
                    perPositionHeight: true,
                },
            })
        }
    }

    function handleCircleMouseMove(movement: Cesium.ScreenSpaceEventHandler.MotionEvent) {
        if (!circleAnchor || !tempCircleEntity) return
        const scene = viewer.scene
        if (!scene.pickPositionSupported) return
        const picked = scene.pickPosition(movement.endPosition)
        if (!picked) return

        currentRadius = Cesium.Cartesian3.distance(circleAnchor, picked)
        const normal = Cesium.Cartesian3.normalize(
            Cesium.Ellipsoid.WGS84.geodeticSurfaceNormal(circleAnchor, new Cesium.Cartesian3()),
            new Cesium.Cartesian3()
        )
        const positions = makeCirclePolygon(circleAnchor, normal, currentRadius)

        tempCircleEntity.polygon!.hierarchy = new Cesium.CallbackProperty(
            () => new Cesium.PolygonHierarchy(positions),
            false
        )

        if (!tempRadiusLine) {
            tempRadiusLine = viewer.entities.add({
                position: new Cesium.CallbackPositionProperty(
                    () => Cesium.Cartesian3.midpoint(circleAnchor!, picked, new Cesium.Cartesian3()),
                    false
                ),
                polyline: {
                    positions: new Cesium.CallbackProperty(() => [circleAnchor!, picked], false),
                    width: 2,
                    material: Cesium.Color.YELLOW,
                },
                label: {
                    text: new Cesium.CallbackProperty(() => `${currentRadius.toFixed(1)} m`, false),
                    font: "14px sans-serif",
                    fillColor: Cesium.Color.YELLOW,
                    showBackground: true,
                    backgroundColor: Cesium.Color.BLACK.withAlpha(0.5),
                    pixelOffset: new Cesium.Cartesian2(0, -10),
                    verticalOrigin: Cesium.VerticalOrigin.TOP,
                },
            })
        }
    }

    function handleCircleLeftClick() {
        if (!circleAnchor || currentRadius <= 0) return

        circleCounter++
        const id = `circle-${circleCounter}`

        const normal = Cesium.Cartesian3.normalize(
            Cesium.Ellipsoid.WGS84.geodeticSurfaceNormal(circleAnchor, new Cesium.Cartesian3()),
            new Cesium.Cartesian3()
        )
        const positions = makeCirclePolygon(circleAnchor, normal, currentRadius)

        const circleEntity = viewer.entities.add({
            polygon: {
                hierarchy: new Cesium.PolygonHierarchy(positions),
                material: Cesium.Color.RED.withAlpha(0.5),
                outline: true,
                outlineColor: Cesium.Color.BLACK,
                perPositionHeight: true,
            },
        })

        const labelEntity = viewer.entities.add({
            position: circleAnchor,
            label: {
                text: `Circle ${circleCounter}`,
                font: '24px sans-serif',
                pixelOffset: new Cesium.Cartesian2(10, -30),
                style: Cesium.LabelStyle.FILL_AND_OUTLINE,
                outlineWidth: 8,
                fillColor: Cesium.Color.WHITE,
                outlineColor: Cesium.Color.BLACK,
                showBackground: false,
                show: true,
            },
        })

        const carto = Cesium.Cartographic.fromCartesian(circleAnchor)
        const point: GeoLocation = {
            lon: Cesium.Math.toDegrees(carto.longitude),
            lat: Cesium.Math.toDegrees(carto.latitude),
            height: carto.height,
        }

        const geoForm: GeoForm = {
            id,
            name: `Circle ${circleCounter}`,
            type: FormType.Circle,
            data: { points: [point], radius: currentRadius },
        }

        geoTools.addForm(geoForm, { polygon: circleEntity, label: labelEntity })

        if (tempCircleEntity) viewer.entities.remove(tempCircleEntity)
        if (tempRadiusLine) viewer.entities.remove(tempRadiusLine)
        tempCircleEntity = null
        tempRadiusLine = null
        circleAnchor = null
        currentRadius = 0
    }

    // ================================
    // Highlighting
    // ================================
    const highlightForm = (e: Cesium.Entity) => {
        if (e.polygon) {
            e.polygon.material = new Cesium.ColorMaterialProperty(Cesium.Color.YELLOW.withAlpha(0.7))
            e.polygon.outlineColor = new Cesium.ColorMaterialProperty(Cesium.Color.WHITE)
        }
        if (e?.label) {
            e.label.fillColor = new Cesium.ColorMaterialProperty(Cesium.Color.YELLOW)
            e.label.font = new Cesium.ConstantProperty("bold 18px sans-serif")
        }
    }

    // ================================
    // Combined event handlers
    // ================================
    handler.setInputAction((movement) => {
        if (geoTools.activeTool === "plane") handlePlaneRightClick(movement)
        else if (geoTools.activeTool === "circle") handleCircleRightClick(movement)
    }, Cesium.ScreenSpaceEventType.RIGHT_CLICK)

    handler.setInputAction((movement) => {
        if (geoTools.activeTool === "plane") handlePlaneMouseMove(movement)
        else if (geoTools.activeTool === "circle") handleCircleMouseMove(movement)
    }, Cesium.ScreenSpaceEventType.MOUSE_MOVE)

    handler.setInputAction(() => {
        if (geoTools.activeTool === "plane") handlePlaneLeftClick()
        else if (geoTools.activeTool === "circle") handleCircleLeftClick()
    }, Cesium.ScreenSpaceEventType.LEFT_CLICK)

    // handler.setInputAction((movement) => {
    //     const pickedObject = viewer.scene.pick(movement.position)
    //     if (!pickedObject) return
    //     const entity = pickedObject.id as Cesium.Entity
    //     highlightForm(entity)
    // }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK)

    return handler
}
