import * as Cesium from "cesium";

let viewer: Cesium.Viewer | null = null;

export function setupDoubleClickHandler(viewer: Cesium.Viewer) {
    const handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
    viewer = viewer;

    handler.setInputAction((click: { position: Cesium.Cartesian2; }) => {

        const pickedObject = viewer.scene.pick(click.position);
        if (Cesium.defined(pickedObject) && pickedObject.id) {
            const entity = pickedObject.id;

            // Check if the entity is selectable (your drones have this property)
            if (entity.properties?.isSelectable?.getValue()) {
                viewer.selectedEntity = entity;

            }
        }
    }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
}