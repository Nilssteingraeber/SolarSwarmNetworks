import { Entity } from "cesium"
import { Raw } from "vue"

export type RobotLocation =
    | {
        lon: number
        lat: number
        alt?: number
    }
    | Record<string, any>


export interface Robot {
    robot_id: number;
    nid: string;

    state_id?: number;
    display_name?: string;

    ipv4?: string;
    ipv6?: string;
    mac?: string;

    battery?: number;
    cpu_1?: number;

    point?: {
        lon: number;
        lat: number;
        alt?: number;
        x: number;
        y: number;
        z?: number;
    } | Record<string, any>;

    orientation?: {
        x?: number
        y?: number
        z?: number
        w?: number
    }


    last_heard?: number;

    connectivity?: Record<string, number>;
}

export interface ProcessedRobot extends Robot {
    time_since_last_heard: number;
    is_stale: boolean;
}

export interface RobotWithEntity {
    robot: Robot;
    entity: Raw<Entity> | undefined;
}
