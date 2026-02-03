<script setup lang="ts">
import { defineProps, defineEmits, ref, watch, onMounted } from 'vue';
import { ServiceInput } from '../../../models/Service';

const props = defineProps<{
    modelValue: string | null,
    serviceParam: ServiceInput
}>();

const emit = defineEmits<{
    (e: 'update:modelValue', value: string | null): void
}>();

const inputValue = ref(props.modelValue);
const inputRef = ref<HTMLInputElement | null>(null);
const spanRef = ref<HTMLSpanElement | null>(null);

// keep internal value in sync with parent
watch(() => props.modelValue, (newVal) => {
    inputValue.value = newVal;
});

watch(inputValue, (newVal) => {
    emit('update:modelValue', newVal);
    resizeInput();
});

function resizeInput() {
    if (!inputRef.value || !spanRef.value) return;

    spanRef.value.textContent = inputValue.value ?? '';
    const textWidth = spanRef.value.offsetWidth;

    // Add buffer: ~18px for spinner + ~8px padding
    inputRef.value.style.width = `${textWidth + 26}px`;
}

onMounted(() => resizeInput());
</script>

<template>
    <div class="bool-background-true d-inline-flex align-items-center rounded-end">
        <input ref="inputRef" :disabled="props.serviceParam.readOnly" type="number" v-model="inputValue"
            class="rounded m-2 small-number-input" />
        <!-- hidden span for measuring -->
        <span ref="spanRef" class="hidden-measurer"></span>
    </div>
</template>

<style scoped>
.small-number-input {
    font-size: 0.8rem;
    border: 1px solid #ccc;
    box-sizing: content-box;
    min-width: 5ch;
    max-width: 30ch;
}

.bool-background-true {
    display: inline-flex;
    width: fit-content;
    align-items: center;
}

.hidden-measurer {
    position: absolute;
    visibility: hidden;
    white-space: pre;
    font-size: 0.8rem;
    font-family: inherit;
    font-weight: inherit;
}
</style>
