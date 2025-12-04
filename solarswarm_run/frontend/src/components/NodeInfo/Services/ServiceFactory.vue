<script setup lang="ts">
import { onMounted, onUnmounted, reactive, ref } from 'vue';
import { Service, DataType } from '../../../models/Service';

// Import your custom components
import TextInput from "./TextInput.vue"
// import FloatInput from './FloatInput.vue';
// import IntegerInput from './IntegerInput.vue';
import BooleanInput from './BooleanInput.vue';
import FloatInput from './FloatInput.vue';
import { MDBCol, MDBRow } from 'mdb-vue-ui-kit';
import { OhVueIcon } from 'oh-vue-icons';
// import CoordinatesInput from './CoordinatesInput.vue';
// import MapCoordinatesInput from './MapCoordinatesInput.vue';

const props = defineProps<{ services: Service[] }>();

// Reactive object to hold input values
const inputs = reactive<Record<string, any>>({});

const currentTime = ref('')
const isShrunk = ref(false);

function toggleLoader() {
    isShrunk.value = !isShrunk.value;
}

function updateTime() {
    const now = new Date()
    const hours = String(now.getHours()).padStart(2, '0')
    const minutes = String(now.getMinutes()).padStart(2, '0')
    const seconds = String(now.getSeconds()).padStart(2, '0')
    currentTime.value = `${hours}:${minutes}:${seconds}`
}

onMounted(() => {
    updateTime() // set initial value
    const interval = setInterval(updateTime, 30000)

    onUnmounted(() => {
        clearInterval(interval)
    })
})

// Initialize defaults
// props.service.parameters.forEach(param => {
//     inputs[param.name] = null;
// });

// Map InputDataType to components
const componentMap = {
    [DataType.Text]: TextInput,
    [DataType.Float]: FloatInput,
    // [InputDataType.Integer]: IntegerInput,
    [DataType.Boolean]: BooleanInput,
    // [InputDataType.Coordinates]: CoordinatesInput,
    // [InputDataType.MapCoordinates]: MapCoordinatesInput,
};

</script>

<template>
    <MDBRow class="g-3">
        <MDBCol v-for="service in props.services" :key="service.name" class="col-12 col-md-6">
            <div class="input-layer p-2 h-100 d-flex flex-column">
                <!-- Service header -->
                <div class="d-flex justify-content-between align-items-center mb-2">
                    <div class="service-name shadowy">{{ service.name }}</div>
                    <div class="flex-grow-1 mx-2">
                        <div class="loader-bar" :class="{ shrink: isShrunk }"></div>
                    </div>
                    <div v-if="service.executable" class="d-flex align-items-center" @click="toggleLoader">
                        <div v-if="!service.immediateExecute" class="execute-button px-1 py-0">
                            <OhVueIcon name="fa-play" scale="1" />
                        </div>
                    </div>
                </div>

                <!-- Service parameters -->
                <div class="d-flex flex-wrap gap-2 mb-2">
                    <div v-for="param in service.parameters" :key="param.name"
                        class="input-layer p-2 d-flex align-items-center flex-grow-1">
                        <div :class="['param-name white-text-outline', { disabled: param.readOnly }]">
                            {{ param.name }}
                        </div>
                        <div class="vertical-rule mx-2"></div>
                        <component :serviceParam="param" :is="componentMap[param.inputDataType]"
                            v-model="inputs[param.name]" :isOutput="false" class="flex-grow-1" />
                    </div>
                </div>

                <!-- Service returns -->
                <div v-if="service.returns?.length" class="d-flex flex-wrap gap-2">
                    <div v-for="returnInstance in service.returns" :key="returnInstance.name"
                        class="input-layer p-2 d-flex align-items-center flex-grow-1">
                        <div class="param-name white-text-outline disabled">
                            {{ returnInstance.name }}
                        </div>
                        <div class="vertical-rule mx-2"></div>
                        <component :serviceParam="returnInstance" :is="componentMap[returnInstance.outputDataType]"
                            v-model="inputs[returnInstance.name]" :isOutput="true" class="flex-grow-1" />
                    </div>
                </div>

                <!-- Timestamp -->
                <div v-if="service.returns?.length" class="mt-auto text-end">
                    <span class="smallest-text bold-text">{{ currentTime }}</span>
                </div>
            </div>
        </MDBCol>
    </MDBRow>
</template>


<style>
.vertical-rule {
    width: 1px;
    height: 100%;
    background-color: #3333330c;
}

.shadowy {
    background: rgba(0, 0, 0, 1.0);
    background-clip: text;
    -webkit-text-fill-color: transparent;
    text-shadow: 0 1px 2px rgba(255, 255, 255, 0.3), 0 0 8px rgba(255, 255, 255, 0.5);
}

.input-layer {
    background-color: rgba(255, 255, 255, 0.350);
    border-radius: 8px;
    border: 1px rgba(126, 126, 126, 0.178) solid;
}

.service-name {
    font-weight: bolder;
    font-size: 14px;
}

.response-text {
    font-weight: normal;
    font-size: small;
}

.disabled {
    color: rgba(46, 46, 46, 0.562);
}

.execute-button {
    cursor: pointer;
    transition: 0.1s ease-in-out;
    color: rgb(46, 46, 46)
}

.execute-button:hover {
    color: rgba(46, 46, 46, 0.685)
}

.smallest-text {
    font-weight: normal;
    font-size: smaller;
}

.bold-text {
    font-weight: bold;
    font-size: medium;
}

.icon-wrapper {
    position: relative;
    width: 2em;
    /* adjust size */
    height: 2em;
}

.icon-wrapper .icon-top,
.icon-wrapper .icon-bottom {
    position: absolute;
    top: 0;
    left: 0;
}

/* Optional: adjust offset to shift one icon slightly */
.icon-top {
    transform: translate(0, 0);
}

.icon-solid-outline {
    filter:
        drop-shadow(1px 0 0 rgba(255, 255, 255, 0.5)) drop-shadow(-1px 0 0 rgba(255, 255, 255, 0.5)) drop-shadow(0 1px 0 rgba(255, 255, 255, 0.5)) drop-shadow(0 -1px 0 rgba(255, 255, 255, 0.5)) drop-shadow(1px 1px 0 rgba(255, 255, 255, 0.5)) drop-shadow(-1px -1px 0 rgba(255, 255, 255, 0.5)) drop-shadow(-1px 1px 0 rgba(255, 255, 255, 0.5)) drop-shadow(1px -1px 0 rgba(255, 255, 255, 0.5));
}


.loader-bar {
    height: 0.5ch;
    border-radius: 4px;
    outline: 2px solid rgb(224, 224, 224);
    background: linear-gradient(90deg,
            #ff34b1 0%,
            #ff8abb 15%,
            #ff9090 30%,
            #ffe66d 55%,
            #ff34b1 100%);
    background-size: 600px 100%;
    /* animation: scrollLover 1s linear infinite; */
    transition:
        transform 0.5s cubic-bezier(0.075, 0.82, 0.165, 1),
        opacity 0.5s cubic-bezier(0.075, 0.82, 0.165, 1);
    transform-origin: center;
    transform: scaleY(1);
}

.loader-bar.shrink {
    transform: scaleY(0);
}

@keyframes scrollLover {
    0% {
        background-position: 0 0;
    }

    100% {
        background-position: -100% 0;
    }
}
</style>