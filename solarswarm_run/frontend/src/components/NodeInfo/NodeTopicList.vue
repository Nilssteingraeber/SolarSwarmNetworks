<script setup>
import { onMounted, ref } from 'vue'
import { Chart, LineController, LineElement, PointElement, LinearScale, Title, CategoryScale } from 'chart.js'
import { MDBCol, MDBRow } from 'mdb-vue-ui-kit'

const { title } = defineProps(['title'])

</script>

<template>
    <MDBRow class="mb-3 w-100 h-100">
        <MDBCol class="text-center status-bar-text-banner-small">
            Hi
        </MDBCol>
    </MDBRow>
</template>

<style scoped>
.rel {
    position: "relative";
}

.white-text-outline {
    text-shadow: -1px -1px 0 #e2e2e2c5, 1px -1px 0 #e2e2e2c5, -1px 1px 0 #e2e2e2c5, 1px 1px 0 #e2e2e2c5;
}

.status-bar-text-banner-small {
    font-size: 12px;
    font-weight: bold;
}
</style>
