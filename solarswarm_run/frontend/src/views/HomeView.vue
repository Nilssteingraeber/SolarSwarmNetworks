<script setup lang="ts">
import TopMenu from '@/components/topMenu/TopMenu.vue';
import Cesium from './Cesium.vue';
import { MDBCol, MDBRow } from 'mdb-vue-ui-kit';
import ConnectionMap from './ConnectionMap.vue';
import { useDroneDataStore } from '../DronesData/DroneDataStore';
import { useDroneEntityStore } from '../DronesData/DroneEntityStore';
import { useDroneHistoryStore } from '../DronesData/DroneHistoryStore';

const droneEntityStore = useDroneEntityStore()
const droneHistoryStore = useDroneHistoryStore()
const droneDataStore = useDroneDataStore()

// const droneFetcher = new DroneFetcher({
//   apiUrl: "",
//   entityStore: droneEntityStore,
//   historyStore: droneHistoryStore,
//   dataStore: droneDataStore      // <-- REQUIRED
// })

// droneFetcher.start()
</script>

<template>

  <div style="width: 100%; height: 100vh">
    <Cesium />
    <ConnectionMap />
    <MDBRow>
      <MDBCol>
        <TopMenu />
      </MDBCol>
    </MDBRow>
  </div>

</template>
