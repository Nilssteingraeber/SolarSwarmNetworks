<script setup lang="ts">
import TopMenu from '@/components/TopMenu/TopMenu.vue';
import Timeline from '../components/TimeLine/Timeline.vue';
import Cesium from './Cesium.vue';
import { MDBCol, MDBRow } from 'mdb-vue-ui-kit';
import ConnectionMap from './ConnectionMap.vue';
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
    <Timeline />
  </div>
</template>
