import * as Cesium from "cesium";
import { GeoForm } from "../models/GeoForm";
export interface EntityBinding {
    polygon?: Cesium.Entity;
    label?: Cesium.Entity;
}
export declare enum SelectedTool {
    None = 0,
    Plane = 1,
    Circle = 2,
    Sphere = 3,
    Line = 4,
    Polygon = 5,
    Move = 6,
    Rotate = 7
}
export declare const useGeoToolsStore: import("pinia").StoreDefinition<"geoTools", Pick<{
    isOpen: import("vue").Ref<boolean, boolean>;
    activeTool: import("vue").Ref<"circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane", "circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane">;
    toggleOpen: () => boolean;
    setOpen: (v: boolean) => boolean;
    setTool: (tool: ("circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane") | SelectedTool) => void;
    clearTool: () => string;
    isActive: (tool: "circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane") => boolean;
    forms: import("vue").Ref<{
        id: string;
        name: string;
        type: import("../models/GeoForm").FormType;
        data: {
            points: {
                lat: number;
                lon: number;
            }[];
            shapeForm?: any;
            radius?: number;
        };
    }[], GeoForm[] | {
        id: string;
        name: string;
        type: import("../models/GeoForm").FormType;
        data: {
            points: {
                lat: number;
                lon: number;
            }[];
            shapeForm?: any;
            radius?: number;
        };
    }[]>;
    entityList: Map<string, EntityBinding>;
    selectedFormId: import("vue").Ref<string, string>;
    deleteForm: (form: Cesium.Entity) => void;
    addForm: (form: GeoForm, entities: EntityBinding) => void;
    selectForm: (id: string | null) => void;
}, "isOpen" | "activeTool" | "forms" | "entityList" | "selectedFormId">, Pick<{
    isOpen: import("vue").Ref<boolean, boolean>;
    activeTool: import("vue").Ref<"circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane", "circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane">;
    toggleOpen: () => boolean;
    setOpen: (v: boolean) => boolean;
    setTool: (tool: ("circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane") | SelectedTool) => void;
    clearTool: () => string;
    isActive: (tool: "circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane") => boolean;
    forms: import("vue").Ref<{
        id: string;
        name: string;
        type: import("../models/GeoForm").FormType;
        data: {
            points: {
                lat: number;
                lon: number;
            }[];
            shapeForm?: any;
            radius?: number;
        };
    }[], GeoForm[] | {
        id: string;
        name: string;
        type: import("../models/GeoForm").FormType;
        data: {
            points: {
                lat: number;
                lon: number;
            }[];
            shapeForm?: any;
            radius?: number;
        };
    }[]>;
    entityList: Map<string, EntityBinding>;
    selectedFormId: import("vue").Ref<string, string>;
    deleteForm: (form: Cesium.Entity) => void;
    addForm: (form: GeoForm, entities: EntityBinding) => void;
    selectForm: (id: string | null) => void;
}, never>, Pick<{
    isOpen: import("vue").Ref<boolean, boolean>;
    activeTool: import("vue").Ref<"circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane", "circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane">;
    toggleOpen: () => boolean;
    setOpen: (v: boolean) => boolean;
    setTool: (tool: ("circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane") | SelectedTool) => void;
    clearTool: () => string;
    isActive: (tool: "circle" | "line" | "polygon" | "none" | "point" | "measure" | "plane") => boolean;
    forms: import("vue").Ref<{
        id: string;
        name: string;
        type: import("../models/GeoForm").FormType;
        data: {
            points: {
                lat: number;
                lon: number;
            }[];
            shapeForm?: any;
            radius?: number;
        };
    }[], GeoForm[] | {
        id: string;
        name: string;
        type: import("../models/GeoForm").FormType;
        data: {
            points: {
                lat: number;
                lon: number;
            }[];
            shapeForm?: any;
            radius?: number;
        };
    }[]>;
    entityList: Map<string, EntityBinding>;
    selectedFormId: import("vue").Ref<string, string>;
    deleteForm: (form: Cesium.Entity) => void;
    addForm: (form: GeoForm, entities: EntityBinding) => void;
    selectForm: (id: string | null) => void;
}, "toggleOpen" | "setOpen" | "setTool" | "clearTool" | "isActive" | "deleteForm" | "addForm" | "selectForm">>;
