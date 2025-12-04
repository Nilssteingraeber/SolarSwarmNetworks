export enum DataType {
    Text,
    Float,
    Integer,
    Boolean,
    Coordinates,
    MapCoordinates,
}

export interface ServiceInput {
    name: string,
    inputDataType: DataType,
    readOnly: boolean,
    iconName?: string,
}

export interface ServiceOutput {
    name: string,
    outputDataType: DataType,
}

export interface Service {
    name: string,
    description?: string,
    executable: boolean,
    immediateExecute: boolean,
    parameters: Array<ServiceInput>,
    returns?: Array<ServiceOutput>,
}