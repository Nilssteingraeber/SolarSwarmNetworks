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
                    <span v-for="(item, i) in items" :key="i" class="wrapper" :ref="el => setLabelRef(el, i)"
                        :class="{ active: selectedIndex === i }">
                        {{ item }}
                    </span>
                </div>
            </div>
        </MDBCol>
    </MDBRow>
</template>

<style scoped></style>
