<script setup lang="ts">
import { defineProps, defineEmits, ref, watch } from 'vue';
import { ServiceInput } from '../../../models/Service';

const props = defineProps<{
    modelValue: string | null,
    serviceParam: ServiceInput
}>();

const emit = defineEmits<{
    (e: 'update:modelValue', value: string | null): void
}>();

const inputValue = ref(props.modelValue);
const isLoading = ref(true);

watch(() => props.modelValue, (newVal) => {
    inputValue.value = newVal;
});

watch(inputValue, (newVal) => {
    emit('update:modelValue', newVal);
});

</script>

<template>
    <div class="bool-background-true h-100 w-100 d-flex align-items-center rounded-end">
        <input :disabled="props.serviceParam.readOnly" type="checkbox" v-model="inputValue" class="rounded m-2" />
    </div>
</template>

<style scoped>
input[type="checkbox"]:hover {
    -webkit-appearance: none;
    -moz-appearance: none;
    appearance: none;
    width: 1rem;
    height: 1rem;
    border: 1px solid #464646;
    border-radius: 16px;
    cursor: pointer;
    position: relative;
    background-color: rgba(0, 0, 0, 0.1);
}


input[type="checkbox"] {
    transition: all 0.5s cubic-bezier(0.075, 0.82, 0.165, 1);
    -webkit-appearance: none;
    -moz-appearance: none;
    appearance: none;
    width: 1rem;
    height: 1rem;
    border: 0.1px solid #464646;
    border-radius: 4px;
    cursor: pointer;
    position: relative;
    background-color: rgb(255, 255, 255);
}

input[type="checkbox"]:checked {
    background-color: #000000b7;
    border-color: #4b4b4b;
}

input[type="checkbox"]:checked::after {
    content: "";
    position: absolute;
    top: 50%;
    left: 50%;
    width: 5px;
    height: 10px;
    border: solid white;
    border-width: 0 2px 2px 0;
    transform: translate(-50%, -60%) rotate(45deg);
}

.bool-background-true {
    background-color: rgba(128, 214, 128, 0);
}
</style>
