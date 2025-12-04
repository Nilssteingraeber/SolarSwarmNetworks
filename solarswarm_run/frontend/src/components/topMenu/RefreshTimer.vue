<template>
    <div class="circular-progress">
        <svg viewBox="0 0 36 36" class="circular-chart">
            <!-- Hintergrund-Kreis -->
            <defs>
                <linearGradient id="rainbowGradient" x1="0%" y1="0%" x2="100%" y2="0%">
                    <stop offset="0%" stop-color="red">
                        <animate attributeName="offset" values="0;1" dur="3s" repeatCount="indefinite" />
                    </stop>
                    <stop offset="20%" stop-color="orange">
                        <animate attributeName="offset" values="0.2;1.2" dur="3s" repeatCount="indefinite" />
                    </stop>
                    <stop offset="40%" stop-color="yellow">
                        <animate attributeName="offset" values="0.4;1.4" dur="3s" repeatCount="indefinite" />
                    </stop>
                    <stop offset="60%" stop-color="green">
                        <animate attributeName="offset" values="0.6;1.6" dur="3s" repeatCount="indefinite" />
                    </stop>
                    <stop offset="80%" stop-color="blue">
                        <animate attributeName="offset" values="0.8;1.8" dur="3s" repeatCount="indefinite" />
                    </stop>
                    <stop offset="100%" stop-color="violet">
                        <animate attributeName="offset" values="1;2" dur="3s" repeatCount="indefinite" />
                    </stop>
                </linearGradient>
            </defs>
            <path class="circle-bg" stroke="url(#rainbowGradient)" d="M18 2.0845
     a 15.9155 15.9155 0 0 1 0 31.831
     a 15.9155 15.9155 0 0 1 0 -31.831" />
            <!-- Vordergrund-Kreis (wird gefüllt) -->
            <path class="circle" :stroke-dasharray="`${progress}, 100`" d="M18 2.0845
           a 15.9155 15.9155 0 0 1 0 31.831
           a 15.9155 15.9155 0 0 1 0 -31.831" />
            <!-- Text -->
        </svg>
    </div>
</template>

<script setup>
import { computed, defineProps } from 'vue'

// Progress wird als Prop übergeben (0 bis 100)
const props = defineProps({
    value: {
        type: Number,
        default: 0
    }
})

// Mappe die Zahl direkt auf das Kreis-Offset
const progress = computed(() => {
    return Math.min(Math.max(props.value, 0), 100)
})
</script>

<style scoped>
.circular-chart {
    padding: 5px;
    margin: 5px;
    width: 48px;
    height: 48px;
    transform: rotate(-90deg);

}

.circle-bg {
    fill: none;
    stroke: #dddddd;
    stroke-width: 3.8;
}

.circle {
    fill: none;
    stroke: #42b883;
    /* Vue Grün */
    stroke-width: 3.8;
    stroke-linecap: round;
    transition: stroke-dasharray 0.35s;
    transform: rotate(0.25turn);
    transform-origin: center;
}
</style>
