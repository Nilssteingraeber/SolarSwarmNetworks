export declare enum FormType {
    PolygonLine = 0,
    Circle = 1,
    Plane = 2,
    Sphere = 3,
    Cylinder = 4,
    Box = 5
}
export interface GeoLocation {
    lat: number;
    lon: number;
}
export interface RelativeLocation {
    x: number;
    y: number;
    z: number;
}
export interface ShapeForm {
    center: RelativeLocation | GeoLocation;
}
export interface BoxForm extends ShapeForm {
    width: number;
    height: number;
    depth: number;
}
export interface SphereForm extends ShapeForm {
    radius: number;
}
export interface CylinderForm extends ShapeForm {
    radius: number;
    height: number;
}
export interface FormData {
    points: Array<RelativeLocation | GeoLocation> | undefined;
    shapeForm: Array<ShapeForm> | undefined;
}
export interface GeoForm {
    id: string;
    name: string;
    type: FormType;
    data: {
        points: GeoLocation[];
        shapeForm?: any;
        radius?: number;
    };
}
