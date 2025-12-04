"use strict";
var __assign = (this && this.__assign) || function () {
    __assign = Object.assign || function(t) {
        for (var s, i = 1, n = arguments.length; i < n; i++) {
            s = arguments[i];
            for (var p in s) if (Object.prototype.hasOwnProperty.call(s, p))
                t[p] = s[p];
        }
        return t;
    };
    return __assign.apply(this, arguments);
};
Object.defineProperty(exports, "__esModule", { value: true });
var vue_1 = require("vue");
var Cesium = require("cesium");
require("cesium/Build/Cesium/Widgets/widgets.css");
var Drone_glb_url_1 = require("@/assets/Robots/Drone.glb?url");
var viewedDroneStore_1 = require("@/stores/viewedDroneStore");
var Robot_1 = require("@/models/Robot");
var dronesStore_1 = require("@/stores/dronesStore");
Cesium.buildModuleUrl.setBaseUrl('/node_modules/cesium/Build/Cesium/');
var cesiumContainer = (0, vue_1.ref)(null);
var _a = (0, viewedDroneStore_1.UseViewedDroneStore)(), currentlyViewedDrone = _a.currentlyViewedDrone, setDrone = _a.setDrone;
var setEntityCollection = (0, dronesStore_1.UseEntityStore)().setEntityCollection;
var GeoTools_1 = require("../components/GeoEditing/GeoTools");
var geoHandler;
// optional cleanup
(0, vue_1.onBeforeUnmount)(function () {
    if (geoHandler)
        geoHandler.destroy();
});
(0, vue_1.onMounted)(function () {
    var viewer = new Cesium.Viewer(cesiumContainer.value, {
        baseLayer: new Cesium.ImageryLayer(new Cesium.OpenStreetMapImageryProvider({
            url: 'http://localhost:8080/styles/basic/512/',
        })),
        baseLayerPicker: false,
        timeline: true,
        animation: true,
        homeButton: false,
        vrButton: false,
        navigationHelpButton: false,
        fullscreenButton: false,
        sceneModePicker: false,
        creditContainer: { appendChild: function () { } },
        requestRenderMode: true,
        shouldAnimate: true,
        geocoder: false,
    });
    // inside onMounted, after viewer is ready:
    geoHandler = (0, GeoTools_1.initGeoToolsHandler)(viewer);
    Cesium.Cesium3DTileset.fromUrl('http://localhost:9000/tileset.json', {
        dynamicScreenSpaceError: true,
        dynamicScreenSpaceErrorDensity: 0.00278,
        dynamicScreenSpaceErrorFactor: 8.0,
        dynamicScreenSpaceErrorHeightFalloff: 0.4,
    }).then(function (tileset) {
        viewer.scene.primitives.add(tileset);
        // tileset.debugShowContentBoundingVolume = true
        // tileset.debugShowBoundingVolume = true
        // tileset.debugShowMemoryUsage = true
        // tileset.debugShowUrl = true
        // viewer.scene.debugShowFramesPerSecond = true
        viewer.targetFrameRate = 120;
        viewer.clock.clockRange = Cesium.ClockRange.CLAMPED;
        viewer.cesiumWidget.screenSpaceEventHandler.removeInputAction(Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
        viewer.cesiumWidget.screenSpaceEventHandler.removeInputAction(Cesium.ScreenSpaceEventType.LEFT_CLICK);
        setEntityCollection(viewer.entities);
        var baseLat = 51.4449;
        var baseLon = 7.2657;
        var drones = [];
        var flightHeight = 180;
        var speed = 10.0;
        var TOTAL_SAMPLES = 3000;
        var SAMPLE_INTERVAL = 1.0;
        var DOT_THRESHOLD = 5;
        var SMOOTH_ALPHA = 0.22;
        var HYSTERESIS_PIXELS = 2;
        var TERRAIN_SAMPLE_INTERVAL = 30.0;
        function getRandomOffset(maxMeters) {
            var latOffset = ((Math.random() - 0.5) * maxMeters) / 111000;
            var lonOffset = ((Math.random() - 0.5) * maxMeters) / (111000 * Math.cos(Cesium.Math.toRadians(baseLat)));
            return [latOffset, lonOffset];
        }
        var startTime = Cesium.JulianDate.now();
        function projectToWindow(cartesian) {
            var scene = viewer.scene;
            if (typeof scene.cartesianToCanvasCoordinates === 'function') {
                try {
                    var res = scene.cartesianToCanvasCoordinates(cartesian);
                    if (res)
                        return res;
                }
                catch (e) { }
            }
            if (Cesium.SceneTransforms &&
                typeof Cesium.SceneTransforms.wgs84ToWindowCoordinates === 'function') {
                try {
                    var res = Cesium.SceneTransforms.wgs84ToWindowCoordinates(scene, cartesian);
                    if (res)
                        return res;
                }
                catch (e) { }
            }
            if (Cesium.SceneTransforms &&
                typeof Cesium.SceneTransforms.wgs84ToDrawingBufferCoordinates === 'function') {
                try {
                    var db = Cesium.SceneTransforms.wgs84ToDrawingBufferCoordinates(scene, cartesian);
                    if (db) {
                        var dpr = window.devicePixelRatio || 1;
                        return new Cesium.Cartesian2(db.x / dpr, db.y / dpr);
                    }
                }
                catch (e) { }
            }
            return null;
        }
        function screenPixelHeightForPosition(positionCartesian, heightMeters) {
            try {
                var baseCarto = Cesium.Cartographic.fromCartesian(positionCartesian);
                var topCarto = new Cesium.Cartographic(baseCarto.longitude, baseCarto.latitude, baseCarto.height + heightMeters);
                var topCartesian = Cesium.Cartesian3.fromRadians(topCarto.longitude, topCarto.latitude, topCarto.height);
                var p0 = projectToWindow(positionCartesian);
                var p1 = projectToWindow(topCartesian);
                if (!p0 || !p1)
                    return 0;
                return Math.abs(p1.y - p0.y);
            }
            catch (_a) {
                return 0;
            }
        }
        function sampleGroundHeightByRay(positionCartesian) {
            return new Promise(function (resolve) {
                var scene = viewer.scene;
                var windowCoord = projectToWindow(positionCartesian);
                if (!windowCoord) {
                    resolve(0);
                    return;
                }
                try {
                    if (scene.pickPositionSupported) {
                        try {
                            var picked = scene.pickPosition(windowCoord);
                            if (picked) {
                                var pickedCarto = Cesium.Cartographic.fromCartesian(picked);
                                resolve(pickedCarto.height);
                                return;
                            }
                        }
                        catch (_a) { }
                    }
                }
                catch (_b) { }
                try {
                    var ray = viewer.scene.camera.getPickRay(windowCoord);
                    if (ray) {
                        var globeIntersection = viewer.scene.globe.pick(ray, viewer.scene);
                        if (globeIntersection) {
                            var carto = Cesium.Cartographic.fromCartesian(globeIntersection);
                            resolve(carto.height);
                            return;
                        }
                    }
                }
                catch (_c) { }
                resolve(0);
            });
        }
        var _loop_1 = function (i) {
            var _a = getRandomOffset(50), latOffset = _a[0], lonOffset = _a[1];
            var lat = baseLat + latOffset;
            var lon = baseLon + lonOffset;
            var heading = Math.random() * 2 * Math.PI;
            var positionProperty = new Cesium.SampledPositionProperty();
            for (var s = 0; s < TOTAL_SAMPLES; s++) {
                var time = Cesium.JulianDate.addSeconds(startTime, s * SAMPLE_INTERVAL, new Cesium.JulianDate());
                var moveDist = speed * SAMPLE_INTERVAL;
                lat += (Math.cos(heading) * moveDist) / 111000;
                lon += (Math.sin(heading) * moveDist) / (111000 * Math.cos(Cesium.Math.toRadians(lat)));
                if (s % 20 === 0)
                    heading = Math.random() * 2 * Math.PI;
                var cartesian = Cesium.Cartesian3.fromDegrees(lon, lat, flightHeight);
                positionProperty.addSample(time, cartesian);
            }
            var droneName = "Drone ".concat(i + 1);
            var id_test = "drone-entity-".concat(i);
            var entity = viewer.entities.add({
                id: id_test,
                name: droneName,
                position: positionProperty,
                model: { uri: Drone_glb_url_1.default, scale: 1.2, show: true },
                label: {
                    text: droneName,
                    font: '12px sans-serif',
                    pixelOffset: new Cesium.Cartesian2(10, -30),
                    style: Cesium.LabelStyle.FILL_AND_OUTLINE,
                    outlineWidth: 8,
                    fillColor: Cesium.Color.WHITE,
                    outlineColor: Cesium.Color.BLACK,
                    showBackground: false,
                    show: true,
                },
                orientation: new Cesium.VelocityOrientationProperty(positionProperty),
                properties: {
                    isSelectable: true,
                    droneData: new Cesium.ConstantProperty(Robot_1.RobotEntity.empty(id_test))
                },
            });
            entity.properties.droneData.getValue().patchData({ nickname: droneName });
            var dotEntity = viewer.entities.add({
                id: "drone-dot-".concat(i),
                position: positionProperty,
                point: { pixelSize: 6, color: Cesium.Color.YELLOW },
                show: false,
            });
            var shadowEntity = viewer.entities.add({
                id: "drone-shadow-".concat(i),
                polyline: {
                    positions: new Cesium.CallbackProperty(function (time, result) {
                        var pos = positionProperty.getValue(time);
                        if (!pos)
                            return undefined;
                        var carto = Cesium.Cartographic.fromCartesian(pos);
                        var groundPos = Cesium.Cartesian3.fromRadians(carto.longitude, carto.latitude, 0.0);
                        if (result && result.length === 2) {
                            result[0] = pos;
                            result[1] = groundPos;
                            return result;
                        }
                        return [pos, groundPos];
                    }, false),
                    width: 4,
                    material: new Cesium.PolylineOutlineMaterialProperty({
                        color: Cesium.Color.GREEN,
                        outlineWidth: 2,
                        outlineColor: Cesium.Color.BLACK,
                    }),
                },
                show: true,
            });
            drones.push({
                entity: entity,
                positionProperty: positionProperty,
                dotEntity: dotEntity,
                shadowEntity: shadowEntity,
                droneName: droneName,
                modelApproxHeightMeters: 4.0,
                smoothedPixelHeight: 0,
                lastDotShown: false,
                lastLabelShown: true,
                lastTerrainSampleJulian: Cesium.JulianDate.addSeconds(startTime, -99999, new Cesium.JulianDate()),
                lastGroundHeight: null,
                lastReportedHeightMeters: null,
            });
        };
        // --- Initialize drones ---
        for (var i = 0; i < 10; i++) {
            _loop_1(i);
        }
        // --- Drone tick updates ---
        viewer.clock.onTick.addEventListener(function (clock) {
            var now = clock.currentTime;
            var anyVisibilityChanged = false;
            var anyLabelChanged = false;
            for (var _i = 0, drones_1 = drones; _i < drones_1.length; _i++) {
                var d = drones_1[_i];
                var currentPos = d.positionProperty.getValue(now);
                if (!currentPos) {
                    d.dotEntity.show = false;
                    d.entity.model.show = true;
                    d.shadowEntity.show = true;
                    continue;
                }
                var pixelHeight = screenPixelHeightForPosition(currentPos, d.modelApproxHeightMeters);
                d.smoothedPixelHeight =
                    d.smoothedPixelHeight === 0
                        ? pixelHeight
                        : d.smoothedPixelHeight + (pixelHeight - d.smoothedPixelHeight) * SMOOTH_ALPHA;
                var shouldShowDot = d.smoothedPixelHeight <= DOT_THRESHOLD;
                var shouldHideDot = d.smoothedPixelHeight >= DOT_THRESHOLD + HYSTERESIS_PIXELS;
                var newDotShown = d.lastDotShown ? !shouldHideDot : shouldShowDot;
                var newModelShown = !newDotShown;
                if (d.lastDotShown !== newDotShown) {
                    d.dotEntity.show = newDotShown;
                    d.lastDotShown = newDotShown;
                    anyVisibilityChanged = true;
                }
                if (d.entity.model.show !== newModelShown) {
                    d.entity.model.show = newModelShown;
                    anyVisibilityChanged = true;
                }
                // if (d.shadowEntity.show !== newModelShown) {
                //     d.shadowEntity.show = newModelShown
                //     anyVisibilityChanged = true
                // }
                // const secondsSinceSample = Cesium.JulianDate.secondsDifference(
                //     now,
                //     d.lastTerrainSampleJulian,
                // )
                // if (newModelShown && secondsSinceSample >= TERRAIN_SAMPLE_INTERVAL) {
                //     d.lastTerrainSampleJulian = Cesium.JulianDate.clone(now)
                //     sampleGroundHeightByRay(currentPos)
                //         .then((groundHeight: number) => {
                //             if (groundHeight === null || typeof groundHeight !== 'number') groundHeight = 0
                //             const cartoNow = Cesium.Cartographic.fromCartesian(currentPos)
                //             const heightAboveGround = cartoNow.height - groundHeight
                //             const rounded = Math.round(heightAboveGround)
                //             if (d.lastReportedHeightMeters !== rounded) {
                //                 d.lastGroundHeight = groundHeight
                //                 d.lastReportedHeightMeters = rounded
                //                 d.entity.label.text = `${d.droneName} — ${rounded} m`
                //                 anyLabelChanged = true
                //             }
                //         })
                //         .catch(() => { })
                // }
            }
            if (anyVisibilityChanged || anyLabelChanged)
                viewer.scene.requestRender();
        });
        var _loop_2 = function (i) {
            var d = drones[i];
            var lineEntity = viewer.entities.add({
                id: "drone-lines-".concat(i),
                polyline: {
                    positions: new Cesium.CallbackProperty(function (time, result) {
                        var positions = [];
                        var currentPos = d.positionProperty.getValue(time);
                        if (!currentPos)
                            return undefined;
                        positions.push(currentPos);
                        for (var j = 1; j <= 3; j++) {
                            var nextIndex = (i + j) % drones.length;
                            var nextDronePos = drones[nextIndex].positionProperty.getValue(time);
                            if (nextDronePos)
                                positions.push(nextDronePos);
                        }
                        if (result) {
                            result.length = 0;
                            for (var _i = 0, positions_1 = positions; _i < positions_1.length; _i++) {
                                var p = positions_1[_i];
                                result.push(p);
                            }
                            return result;
                        }
                        return positions;
                    }, false),
                    width: 2,
                    material: Cesium.Color.CYAN.withAlpha(0.7),
                    clampToGround: false,
                },
            });
            d.lineEntity = lineEntity;
        };
        // --- Drone connecting lines ---
        for (var i = 0; i < drones.length; i++) {
            _loop_2(i);
        }
        viewer.scene.camera.flyTo({
            destination: Cesium.Cartesian3.fromDegrees(baseLon, baseLat, flightHeight + 1000),
            duration: 1,
        });
    });
    var centerNRW = Cesium.Cartesian3.fromDegrees(7.5, 51.433, 81000);
    viewer.camera.setView({
        destination: centerNRW,
        orientation: { heading: Cesium.Math.toRadians(0), pitch: Cesium.Math.toRadians(-30), roll: 0 },
    });
    var handler = new Cesium.ScreenSpaceEventHandler(viewer.canvas);
    handler.setInputAction(function (click) {
        var _a;
        var pickedObject = viewer.scene.pick(click.position);
        if (!Cesium.defined(pickedObject) || !(pickedObject === null || pickedObject === void 0 ? void 0 : pickedObject.id))
            return;
        var entity = pickedObject.id;
        // Only fly to entities that are selectable
        if (Cesium.defined((_a = entity.properties) === null || _a === void 0 ? void 0 : _a.isSelectable) &&
            entity.properties.isSelectable.getValue()) {
            entity.properties.isSelected = true;
            setDrone(entity.id);
        }
    }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
});
debugger; /* PartiallyEnd: #3632/scriptSetup.vue */
var __VLS_ctx = {};
var __VLS_components;
var __VLS_directives;
__VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ ref: "cesiumContainer" }, { style: {} }));
/** @type {typeof __VLS_ctx.cesiumContainer} */ ;
var __VLS_dollars;
var __VLS_self = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {
            cesiumContainer: cesiumContainer,
        };
    },
});
exports.default = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {};
    },
});
; /* PartiallyEnd: #4569/main.vue */
