export interface RoboterBaseData {
    nickname: string;
    readonly nid: string;
    activity: string;
    battery: number;
    cpu: number;
    misc: string;
    conn_strength: number;
    pose: Record<string, any>;
    services: Record<string, any>;
}
export declare class RobotEntity {
    readonly data: RoboterBaseData;
    constructor(data: RoboterBaseData);
    static empty: (nid: string) => RobotEntity;
    patchData(incomingData: Partial<RoboterBaseData>): void;
}
