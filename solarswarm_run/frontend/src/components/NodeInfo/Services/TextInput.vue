<script setup lang="ts">
import { Service, ServiceOutput } from '@/models/Service';
import { MDBInput, MDBTextarea } from 'mdb-vue-ui-kit';
import { defineProps, defineEmits, ref, watch, onMounted, nextTick } from 'vue';

const props = defineProps<{
    modelValue: string | null,
    returnInstance: Service,
    isOutput: boolean,
    readOnly?: boolean // <-- added
}>();

const emit = defineEmits<{
    (e: 'update:modelValue', value: string | null): void
}>();

const inputValue = ref(props.modelValue);

// Keep internal value in sync with parent
watch(() => props.modelValue, (newVal) => {
    inputValue.value = newVal;
});

// Emit changes to parent
watch(inputValue, (newVal) => {
    emit('update:modelValue', newVal);
});

// Ref to the textarea
const textareaRef = ref<HTMLElement | null>(null);

// Auto-resize function: horizontal first, then vertical
const autoResize = () => {
    if (!textareaRef.value) return;

    const textarea = textareaRef.value.querySelector('textarea') as HTMLTextAreaElement;
    if (!textarea) return;

    // Reset height to auto to shrink if needed
    textarea.style.height = 'auto';

    // Set width up to max
    const parentWidth = textarea.parentElement?.offsetWidth || 0;
    textarea.style.width = 'auto'; // grow naturally
    textarea.style.width = Math.min(textarea.scrollWidth, parentWidth) + 'px';

    // Once width is full, grow height
    if (textarea.offsetWidth >= parentWidth) {
        textarea.style.height = textarea.scrollHeight + 'px';
    }
};
</script>

<template>
    <div class="m-2" ref="textareaRef">
        <MDBTextarea type="textarea" v-model="inputValue" class="rounded resizable-input"
            :disabled="isOutput || readOnly" rows="1" @input="autoResize" />
    </div>
</template>

<style scoped>
.resizable-input textarea {
    overflow: hidden;
    /* hide scrollbar */
    resize: none;
    /* JS handles resizing */
    min-width: 100px;
    max-width: 100%;
    transition: width 0.2s ease-in-out, height 0.2s ease-in-out;
    /* optional smooth animation */
}
</style>
