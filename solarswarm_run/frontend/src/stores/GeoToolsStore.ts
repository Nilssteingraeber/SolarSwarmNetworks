// src/stores/GeoToolsStore.ts
import { defineStore } from "pinia"
import { ref } from "vue"
import * as Cesium from "cesium"
import { GeoForm } from "../models/GeoForm"
import { geoWorkerService } from "../components/GeoEditing/GeoWorkerServices"

export interface EntityBinding {
    primary: Cesium.Entity
    labels: Cesium.Entity[]
    outline?: Cesium.Entity
}

export enum SelectedTool {
    None, Marker, Plane, Circle, Sphere, Line, Polygon, Move, Rotate, Delete
}

export const useGeoToolsStore = defineStore("geoTools", () => {
    const isOpen = ref(false)
    const activeTool = ref<SelectedTool>(SelectedTool.None)
    const forms = ref<GeoForm[]>([])
    // Map ID -> Cesium Entities (so we can remove them from screen)
    const entityList = new Map<string, EntityBinding>()

    const toggleOpen = () => (isOpen.value = !isOpen.value)
    const setOpen = (v: boolean) => (isOpen.value = v)
    const setTool = (tool: SelectedTool) => activeTool.value = tool

    // --- Actions ---

    const addForm = (form: GeoForm, entities: EntityBinding) => {
        forms.value.push(form)
        entityList.set(form.id, entities)
        // Persist via Worker
        geoWorkerService.saveShape(form)
    }

    const deleteForm = (id: string, viewer: Cesium.Viewer) => {
        const binding = entityList.get(id)
        if (binding) {
            viewer.entities.remove(binding.primary)
            binding.labels.forEach(l => viewer.entities.remove(l))
            if (binding.outline) viewer.entities.remove(binding.outline)
        }
        entityList.delete(id)
        forms.value = forms.value.filter(f => f.id !== id)

        // Remove from DB via Worker
        geoWorkerService.deleteShape(id)
    }

    const clearAll = (viewer: Cesium.Viewer) => {
        entityList.forEach((binding) => {
            viewer.entities.remove(binding.primary);
            binding.labels.forEach(l => viewer.entities.remove(l));
        });
        entityList.clear();
        forms.value = [];
        geoWorkerService.clearAll();
    }

    const loadFromDisk = async () => {
        const loaded = await geoWorkerService.getAllShapes();
        forms.value = loaded;
        return loaded; // Return to component so it can draw them
    }

    const exportToJson = () => {
        const dataStr = JSON.stringify(forms.value, null, 2);
        const blob = new Blob([dataStr], { type: "application/json" });
        const url = URL.createObjectURL(blob);
        const link = document.createElement('a');
        link.href = url;
        link.download = `geotools_export.json`;
        link.click();
        URL.revokeObjectURL(url);
    }

    return {
        isOpen, activeTool, forms, entityList,
        toggleOpen, setOpen, setTool,
        addForm, deleteForm, clearAll, loadFromDisk, exportToJson
    }
})