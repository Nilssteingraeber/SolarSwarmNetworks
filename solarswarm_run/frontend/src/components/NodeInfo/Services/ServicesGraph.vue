<script setup lang="ts">
import { onMounted, ref } from 'vue'
import { Chart, LineController, LineElement, PointElement, LinearScale, Title, CategoryScale } from 'chart.js'
import { MDBCol, MDBRow } from 'mdb-vue-ui-kit'
import ServiceFactory from './ServiceFactory.vue';
import { MockServices } from '@/mock/MockServices';
</script>

<template>
    <ServiceFactory :services="MockServices" />
</template>

<style scoped>
.rel {
    position: "relative";
}

.loader {
    width: 800px;
    height: 100px;
    border-radius: 0px;
    background: linear-gradient(90deg,
            #ff9a9e 0%,
            #fad0c4 25%,
            #fffb00 50%,
            #fbc2eb 75%,
            #a18cd1 90%,
            #ff9a9e 100%);
    background-size: 200px 100%;
    /* size of one full repeating pattern */
    animation: scrollLover 5s linear infinite;
}

@keyframes scrollLover {
    0% {
        background-position: 0 0;
        /* start at left edge */
    }

    100% {
        background-position: 600px 0;
        /* move left by exactly one pattern width */
    }
}

.gradient-text {
    /* adjust as needed */
    font-weight: bold;
    background: linear-gradient(90deg,
            #555 0%,
            #555 25%,
            #999 50%,
            #555 75%,
            #555 100%);
    background-size: 200px 100%;
    -webkit-background-clip: text;
    /* Chrome/Safari */
    -webkit-text-fill-color: transparent;
    /* Chrome/Safari */
    background-clip: text;
    /* Other browsers */
    color: transparent;
    /* fallback */
    animation: scrollLover 5s linear infinite;
}

@keyframes scrollLover {
    0% {
        background-position: 0 0;
    }

    100% {
        background-position: 600px 0;
    }
}
</style>
