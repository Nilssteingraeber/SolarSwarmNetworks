<script setup lang="ts">
import { MDBTable } from 'mdb-vue-ui-kit'
import { storeToRefs } from 'pinia'
import GeoItemListEntry from './GeoItemListEntry.vue'
import { BoundingSphere, Cartesian3, HeadingPitchRange, Math as CesiumMath } from 'cesium'
import { useDroneEntityStore } from '../../DronesData/DroneEntityStore'
import { GeoForm } from '../../models/GeoForm'
import { useGeoToolsStore, SelectedTool } from '../../stores/GeoToolsStore'

const geoStore = useGeoToolsStore()
const droneEntityStore = useDroneEntityStore();
const { forms } = storeToRefs(geoStore)

const onLocate = (form: GeoForm) => {
    if (!droneEntityStore.viewer) return;

    // Robust way to find center for ANY shape (Line, Poly, Sphere, Point)
    // We rely on the raw data points, not the Cesium Entity type
    const cartesianPoints = form.data.points.map(p =>
        Cartesian3.fromDegrees(p.lon, p.lat, p.height ?? 0)
    )

    if (cartesianPoints.length === 0) return

    const boundingSphere = BoundingSphere.fromPoints(cartesianPoints)

    // Add radius padding for single points (Circle/Sphere centers)
    if (cartesianPoints.length === 1 && form.data.radius) {
        boundingSphere.radius = form.data.radius;
    }

    droneEntityStore.viewer.camera.flyToBoundingSphere(boundingSphere, {
        duration: 1.5,
        offset: new HeadingPitchRange(
            CesiumMath.toRadians(0),   // Heading
            CesiumMath.toRadians(-45), // Pitch (looking down at angle)
            boundingSphere.radius * 2.5 || 200 // Range (default 200m if radius is 0)
        )
    })
}

const onEdit = (form: GeoForm) => {
    // Basic implementation: Close list, switch to Move tool
    // Note: Selecting specific item for edit requires more complex state in Store (selectedId)
    console.log("Edit form", form.name)
    geoStore.setTool(SelectedTool.Move)
}

const onDelete = (form: GeoForm) => {
    if (droneEntityStore.viewer) {
        geoStore.deleteForm(form.id, droneEntityStore.viewer)
    }
}
</script>

<template>
    <div class="table-container">
        <MDBTable hover small class="mb-0">
            <thead>
                <tr>
                    <th scope="col">#</th>
                    <th scope="col">Name</th>
                    <th scope="col">Type</th>
                    <th scope="col" class="text-end">Actions</th>
                </tr>
            </thead>
            <tbody>
                <tr v-for="(form, index) in forms" :key="form.id">
                    <th scope="row" class="align-middle">{{ index + 1 }}</th>
                    <td class="align-middle text-truncate" style="max-width: 100px;">{{ form.name }}</td>
                    <td class="align-middle">{{ form.type }}</td>
                    <td class="text-end">
                        <GeoItemListEntry :form="form" @locate="onLocate" @edit="onEdit" @delete="onDelete" />
                    </td>
                </tr>
                <tr v-if="forms.length === 0">
                    <td colspan="4" class="text-center text-muted p-3">
                        <small>No shapes added yet.</small>
                    </td>
                </tr>
            </tbody>
        </MDBTable>
    </div>
</template>

<style scoped>
.table-container {
    max-height: 40vh;
    /* Prevent list from growing too large */
    overflow-y: auto;
}
</style>