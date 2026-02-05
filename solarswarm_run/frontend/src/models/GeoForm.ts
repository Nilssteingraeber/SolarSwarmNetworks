export enum FormType {
    Marker = "Marker",
    Point = "Point",
    LineString = "LineString",
    Polygon = "Polygon",
    Circle = "Circle",
    Sphere = "Sphere",
    Plane = "Plane"
}

export interface GeoLocation {
    lon: number;
    lat: number;
    height: number;
}

export interface GeoForm {
    id: string;
    name: string;
    type: FormType;
    data: {
        points: GeoLocation[];
        radius?: number;
    };
}

export interface RelativeLocation {
    x: number,
    y: number,
    z: number,
}

export interface ShapeForm {
    center: RelativeLocation | GeoLocation
}

export interface BoxForm extends ShapeForm {
    width: number,
    height: number,
    depth: number
}

export interface SphereForm extends ShapeForm {
    radius: number
}

export interface CylinderForm extends ShapeForm {
    radius: number,
    height: number
}

export interface FormData {
    points: Array<RelativeLocation | GeoLocation> | undefined
    shapeForm: Array<ShapeForm> | undefined
}

