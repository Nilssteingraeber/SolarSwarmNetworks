export declare enum DataType {
    Text = 0,
    Float = 1,
    Integer = 2,
    Boolean = 3,
    Coordinates = 4,
    MapCoordinates = 5
}
export interface ServiceInput {
    name: string;
    inputDataType: DataType;
    readOnly: boolean;
    iconName?: string;
}
export interface ServiceOutput {
    name: string;
    outputDataType: DataType;
}
export interface Service {
    name: string;
    description?: string;
    executable: boolean;
    immediateExecute: boolean;
    parameters: Array<ServiceInput>;
    returns?: Array<ServiceOutput>;
}
