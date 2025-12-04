<script setup lang="ts">
import { MDBTable } from 'mdb-vue-ui-kit'
import { useGeoToolsStore } from '../../stores/GeoToolsStore'
import { storeToRefs } from 'pinia'
import GeoItemListEntry from './GeoItemListEntry.vue'
import { useDroneEntityStore } from '../../DronesData/DroneEntityStore'
import { GeoForm } from '../../models/GeoForm'
import { toRaw } from 'vue'
import { BoundingSphere, Cartesian3, Color, HeadingPitchRange, Math } from 'cesium'

const geoStore = useGeoToolsStore()
const droneEntityStore = useDroneEntityStore();

const { forms } = storeToRefs(geoStore)

const onLocate = (form: GeoForm) => {
    console.log("Locate form", form)

    const entity = geoStore.entityList.get(form.id)
    if (entity?.polygon) {
        const cartesianPoints = form.data.points.map(p =>
            Cartesian3.fromDegrees(p.lon, p.lat, p.height ?? 0)
        )

        if (cartesianPoints.length === 0) return

        const boundingSphere = BoundingSphere.fromPoints(cartesianPoints)

        droneEntityStore.viewer?.camera.flyToBoundingSphere(boundingSphere, {
            duration: 1,
            offset: new HeadingPitchRange(
                Math.toRadians(0), // heading
                Math.toRadians(-30), // pitch
                boundingSphere.radius * 1.2 // distance multiplier for some padding
            )
        })
    }


}

const onEdit = (form: any) => {
    console.log("Edit form", form)
    // TODO: Open editing UI
}

const onDelete = (form: any) => {
    console.log("Delete form", form)
    geoStore.deleteForm(form)
}
</script>

<template>
    <MDBTable hover>
        <thead>
            <tr>
                <th scope="col">#</th>
                <th scope="col">Name</th>
                <th scope="col">Type</th>
                <th scope="col">Tools</th>
            </tr>
        </thead>
        <tbody>
            <tr v-for="(form, index) in forms" :key="form.id">
                <th scope="row">{{ index + 1 }}</th>
                <td>{{ form.name }}</td>
                <td>{{ form.type }}</td>
                <td>
                    <GeoItemListEntry :form="form" @locate="onLocate" @edit="onEdit" @delete="onDelete" />
                </td>
            </tr>
            <tr v-if="forms.length === 0">
                <td colspan="4" class="text-center">No forms created yet.</td>
            </tr>
        </tbody>
    </MDBTable>
</template>
