import { defineStore } from "pinia"
import { ref } from "vue"
import * as Cesium from "cesium"
import { GeoForm, GeoLocation } from "../models/GeoForm"

// --- Entity bindings ---
export interface EntityBinding {
    polygon?: Cesium.Entity
    label?: Cesium.Entity
}

// --- Generic Tool Enum ---
export enum SelectedTool {
    None,
    Plane,
    Circle,
    Sphere,
    Line,
    Polygon,
    Move,
    Rotate,
}

// --- Map enum to Cesium-compatible tool strings ---
const toolToCesiumMap: Record<
    SelectedTool,
    "none" | "point" | "line" | "polygon" | "circle" | "measure" | "plane"
> = {
    [SelectedTool.None]: "none",
    [SelectedTool.Plane]: "plane",
    [SelectedTool.Circle]: "circle",
    [SelectedTool.Sphere]: "measure",
    [SelectedTool.Line]: "line",
    [SelectedTool.Polygon]: "polygon",
    [SelectedTool.Move]: "none",
    [SelectedTool.Rotate]: "none",
}

export const useGeoToolsStore = defineStore("geoTools", () => {
    // --- UI state ---
    const isOpen = ref(false)
    const activeTool = ref<
        "none" | "point" | "line" | "polygon" | "circle" | "measure" | "plane"
    >("circle")

    // --- Forms ---
    const forms = ref<GeoForm[]>([])
    const entityList = new Map<string, EntityBinding>()
    const selectedFormId = ref<string | null>(null)

    // --- Actions ---
    const toggleOpen = () => (isOpen.value = !isOpen.value)
    const setOpen = (v: boolean) => (isOpen.value = v)

    // Accept either literal string or SelectedTool enum
    const setTool = (tool: typeof activeTool.value | SelectedTool) => {
        if (typeof tool === "number") {
            activeTool.value = toolToCesiumMap[tool]
        } else {
            activeTool.value = tool
        }
    }

    const clearTool = () => (activeTool.value = "circle")
    const isActive = (tool: typeof activeTool.value) => activeTool.value === tool

    const addForm = (form: GeoForm, entities: EntityBinding) => {
        forms.value.push(form)
        entityList.set(form.id, entities)
    }

    const deleteForm = (form: Cesium.Entity) => {
        entityList.delete(form.id)
        forms.value = forms.value.filter(f => f.id !== form.id)
    }

    const selectForm = (id: string | null) => {
        selectedFormId.value = id
    }

    const computeGeoCenter = (points: GeoLocation[]) => {
        return Cesium.BoundingSphere.fromPoints(points.map(p =>
            Cesium.Cartesian3.fromDegrees(p.lon, p.lat, p.height)
        )).center;
    }


    return {
        isOpen,
        activeTool,
        toggleOpen,
        setOpen,
        setTool,
        clearTool,
        isActive,

        forms,
        entityList,
        selectedFormId,
        deleteForm,
        addForm,
        selectForm,
        computeGeoCenter,
    }
})
