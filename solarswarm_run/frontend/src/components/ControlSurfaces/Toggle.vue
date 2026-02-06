<script setup lang="ts">
import { MDBCol, MDBRow } from 'mdb-vue-ui-kit';
import { ref, defineProps, computed, onMounted, nextTick, onBeforeUnmount, watch } from 'vue';

const props = defineProps<{ label: string, toggleItems?: string[] }>();

const items = computed(() =>
    props.toggleItems && props.toggleItems.length >= 2
        ? props.toggleItems.slice(0, 2)
        : ["Geographic", "Cartesian"]
);

const selectedIndex = ref(0);
const toggle = () => (selectedIndex.value = selectedIndex.value === 0 ? 1 : 0);

const labelRefs = ref<HTMLElement[]>([]);
const trackRef = ref<HTMLElement | null>(null);

const setLabelRef = (el: HTMLElement | null, i: number) => {
    if (el) labelRefs.value[i] = el;
};

const indicatorStyle = ref({ left: '0px', width: '0px' });
const indicatorExtra = 2; // total extra width for the pill

const updateIndicator = () => {
    if (!trackRef.value) return;
    const trackRect = trackRef.value.getBoundingClientRect();
    const el = labelRefs.value[selectedIndex.value];
    if (!el) return;
    const rect = el.getBoundingClientRect();
    const left = rect.left - trackRect.left;
    const width = rect.width + indicatorExtra;
    indicatorStyle.value = { left: left + 'px', width: width + 'px' };
};

const handleResize = () => nextTick(updateIndicator);

onMounted(() => {
    nextTick(updateIndicator);
    window.addEventListener('resize', handleResize);
});

onBeforeUnmount(() => {
    window.removeEventListener('resize', handleResize);
});

watch(selectedIndex, () => nextTick(updateIndicator));
</script>

<template>
    <MDBRow class="toggle-wrapper d-flex">
        <MDBCol class="col-auto d-flex align-items-center text-label-before shadowy pe-0">
            {{ label }}
        </MDBCol>
        <MDBCol class="col d-flex justify-content-end ps-0">
            <div class="toggle-track" ref="trackRef" @click="toggle">
                <div class="toggle-indicator" :style="indicatorStyle"></div>
                <div class="labels">
                    <span v-for="(item, i) in items" :key="i" class="wrapper" :ref="el => setLabelRef(el as any, i)"
                        :class="{ active: selectedIndex === i }">
                        {{ item }}
                    </span>
                </div>
            </div>
        </MDBCol>
    </MDBRow>
</template>

<style scoped>
.text-label-before {
    font-size: 10px;
    font-weight: bold;
}

.toggle-wrapper {
    display: flex;
    flex-wrap: nowrap;
    /* prevent wrapping */
    align-items: center;
    justify-content: space-between;
    /* max spacing between label and toggle */
    gap: 8px;
    /* spacing on small screens */
    font-size: 10px;
    font-weight: lighter;
}

.toggle-track {
    position: relative;
    display: flex;
    align-items: center;
    border-radius: 16px;
    background: #f1f1f163;
    cursor: pointer;
    overflow: hidden;
    user-select: none;
    padding: 0px 6px;
    max-height: 5ch;
    outline: 1px dashed rgba(0, 0, 0, 0.123);
}

.shadowy {
    background: rgba(0, 0, 0, 1.0);
    background-clip: text;
    -webkit-text-fill-color: transparent;
    text-shadow: 0 1px 2px rgba(255, 255, 255, 0.3), 0 0 8px rgba(255, 255, 255, 0.5);
}

.toggle-indicator {
    position: absolute;
    top: 50%;
    transform: translateY(-50%);
    height: 66.6%;
    background: #007bff;
    border-radius: 8px;
    transition: left 0.3s ease, width 0.3s ease;
    z-index: 1;
}

.labels {
    display: flex;
    position: relative;
    z-index: 2;
}

.wrapper {
    flex: none;
    padding: 0 8px;
    text-align: center;
    line-height: 32px;
    font-weight: bold;
    color: rgb(116, 116, 116);
    transition: color 0.3s ease;
    white-space: nowrap;
}

.wrapper.active {
    font-weight: bold;
    color: rgb(250, 250, 250);
}

.input-layer {
    background-color: rgba(255, 255, 255, 0.350);
    border-radius: 8px;
    border: 1px rgba(126, 126, 126, 0.178) solid;
}
</style>
