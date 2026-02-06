<script setup lang="ts">
import { MDBTextarea } from 'mdb-vue-ui-kit';
import { defineProps, defineEmits, ref, watch } from 'vue';
import { Service } from '../../../models/Service';

const props = defineProps<{
    modelValue: string | null,
    returnInstance: Service,
    isOutput: boolean,
    readOnly?: boolean
}>();

const inputValue = ref(props.modelValue);

const emit = defineEmits<{
    (e: 'update:modelValue', value: string | null): void
}>();

watch(inputValue, (newVal) => {
    emit('update:modelValue', newVal);
});

watch(() => props.modelValue, (newVal) => {
    inputValue.value = newVal;
});

const textareaRef = ref<HTMLElement | null>(null);

const autoResize = () => {
    if (!textareaRef.value) return;

    const textarea = textareaRef.value.querySelector('textarea') as HTMLTextAreaElement;
    if (!textarea) return;

    textarea.style.height = 'auto';
    const parentWidth = textarea.parentElement?.offsetWidth || 0;
    textarea.style.width = 'auto';
    textarea.style.width = Math.min(textarea.scrollWidth, parentWidth) + 'px';

    if (textarea.offsetWidth >= parentWidth) {
        textarea.style.height = textarea.scrollHeight + 'px';
    }
};
</script>

<template>
    <div class="m-2" ref="textareaRef">
        <MDBTextarea type="textarea" :v-model="(String(inputValue))" class="rounded resizable-input"
            :disabled="isOutput || readOnly" rows="1" @input="autoResize" />
    </div>
</template>

<style scoped>
.resizable-input textarea {
    overflow: hidden;
    resize: none;
    min-width: 100px;
    max-width: 100%;
    transition: width 0.2s ease-in-out, height 0.2s ease-in-out;
}
</style>
