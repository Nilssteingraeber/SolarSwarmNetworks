// types.ts

export interface Robot {
    robot_id: number;
    nid: string;
    state_id: number;
    display_name: string;
    ipv4?: string;
    ipv6?: string;
    mac?: string;

    statuses?: Status[];
    state_changes?: StateChange[];
    state?: State;
}

export interface Status {
    status_id: number;
    robot_id: number;
    battery?: number;
    cpu_1?: number;
    point?: Record<string, any>;
    orientation?: Record<string, any>;
    last_heard?: number;

    robot?: Robot;
}

export interface Neighbor {
    robot_id: number;
    neighbor: number;
    strength?: number;

    robot?: Robot;
    neighbor_robot?: Robot;
}

export interface State {
    state_id: number;
    state: string;

    robots?: Robot[];
    state_changes?: StateChange[];
}

export interface StateChange {
    change_id: number;
    state_id: number;
    robot_id: number;
    begin?: number;

    state?: State;
    robot?: Robot;
}
