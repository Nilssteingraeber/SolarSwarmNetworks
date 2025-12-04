<script setup>
import { ref } from 'vue'
import { OhVueIcon } from 'oh-vue-icons';

const holding = ref(false)
const off = ref(false)

let holdTimer_first = null
let holdTimer_second = null
let holdTimer_third = null

const holdDuration = 2000

const powerButton_first_step = ref(false)
const powerButton_second_step = ref(false)
const powerButton_third_step = ref(false)

function startHold() {
    if (off.value) return
    holding.value = true
    powerButton_first_step.value = true

    holdTimer_first = setTimeout(() => {

        powerButton_first_step.value = false
        powerButton_second_step.value = true

        holdTimer_second = setTimeout(() => {

            powerButton_third_step.value = true
            powerButton_second_step.value = false

            holdTimer_third = setTimeout(() => {

                powerButton_third_step.value = false

                holding.value = false
                off.value = false
                alert('Power Off!')
            }, holdDuration)
        }, holdDuration)
    }, holdDuration)
}

function cancelHold() {
    holding.value = false
    powerButton_first_step.value = false
    powerButton_second_step.value = false
    powerButton_third_step.value = false
    clearTimeout(holdTimer_first)
    clearTimeout(holdTimer_second)
    clearTimeout(holdTimer_third)
}
</script>

<template>
    <MDBCol class="input-layer mx-2 mb-2 col-auto d-flex align-items-center p-2">
        <div class="power-button"
            :class="{ holding, off, powerButton_first_step, powerButton_second_step, powerButton_third_step }"
            @mousedown="startHold" @mouseup="cancelHold" @mouseleave="cancelHold" @touchstart.prevent="startHold"
            @touchend="cancelHold" @touchcancel="cancelHold">
            <OhVueIcon name="io-power" scale="1.5" class="pr-2"></OhVueIcon>
        </div>
    </MDBCol>
</template>

<style scoped>
.power-button {
    /* color: #4f4f4f; */
    position: relative;
    cursor: pointer;
    transition: all ease-in-out 0.3s
}

.power-button.powerButton_third_step {
    color: rgb(255, 0, 0);
    transform: scale(0.60);
}


.power-button.powerButton_second_step {
    color: rgb(255, 136, 0);
    transform: scale(0.80);
}

.power-button.powerButton_first_step {
    color: rgb(207, 179, 20);
    transform: scale(0.90);
}

.power-button.off {
    border-color: #f00;
}
</style>
