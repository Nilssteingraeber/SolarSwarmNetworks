<script setup lang="ts">
import type { GeoForm } from "@/models/GeoForm"
import { OhVueIcon } from "oh-vue-icons";
import { defineProps, defineEmits } from "vue"

const props = defineProps<{
    form: GeoForm
}>()

const emit = defineEmits<{
    (e: "locate", form: GeoForm): void
    (e: "edit", form: GeoForm): void
    (e: "delete", form: GeoForm): void
}>()

const locate = () => emit("locate", props.form)
const edit = () => emit("edit", props.form)
const remove = () => emit("delete", props.form)
</script>

<template>
    <div class="actions">
        <button class="btn p-1 mx-1 std-button" @click="locate">
            <OhVueIcon name="md-mylocation-twotone" />
        </button>
        <button class="btn p-1 mx-1 std-button" @click="edit">
            <OhVueIcon name="fa-regular-edit" />
        </button>
        <button class="btn p-1 mx-1 std-button" @click="remove">
            <OhVueIcon class="red" name="bi-trash" />
        </button>
    </div>
</template>

<style scoped>
.actions {
    white-space: nowrap;
}

button {
    font-size: 0.7rem;
    padding: 0.15rem 0.4rem;
}

.red {
    color: #ff4040
}

.std-button:hover {
    color: rgb(29, 132, 228);
    outline: solid 1px rgba(228, 228, 228, 0.936);
    box-shadow: rgba(83, 83, 83, 0.329) 0px 3px 4px;
    background-color: #ffffff44;
}

.std-button {
    outline: solid 1px rgba(228, 228, 228, 0.936);
    box-shadow: rgba(83, 83, 83, 0.329) 0px 3px 4px;
    background-color: #ffffff44;
}
</style>
