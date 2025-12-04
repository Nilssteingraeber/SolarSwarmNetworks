// StateController.ts
import type { Robot, Status, Neighbor, State, StateChange } from "./types";
import * as Networks from "./Networks";

/**
 * GraphNode represents a robot in the topology
 */
export interface GraphNode {
    robot: Robot;
    neighbors: Map<number, number>; // neighbor robot_id -> strength
    active: boolean;
}

/**
 * StateController
 * Periodically collects robots, statuses, neighbors, states, and builds topology.
 */
export class StateController {
    private robots: Map<number, Robot> = new Map();
    private statuses: Map<number, Status> = new Map();
    private neighbors: Map<string, Neighbor> = new Map(); // key: robotId-neighborId
    private states: Map<number, State> = new Map();
    private stateChanges: Map<number, StateChange> = new Map();

    private topology: Map<number, GraphNode> = new Map();

    private intervalId: ReturnType<typeof setInterval> | null = null;
    private intervalMs: number;

    constructor(intervalMs = 1000) {
        this.intervalMs = intervalMs;
    }

    // ----------------------------
    // Start periodic fetching
    // ----------------------------
    public start(): void {
        if (this.intervalId) return; // already running

        this.intervalId = setInterval(() => {
            this.collectFrame().catch((err) => console.error("Error fetching frame:", err));
        }, this.intervalMs);

        this.collectFrame().catch((err) => console.error("Error fetching frame:", err));
    }

    // ----------------------------
    // Stop periodic fetching
    // ----------------------------
    public stop(): void {
        if (this.intervalId) {
            clearInterval(this.intervalId);
            this.intervalId = null;
        }
    }

    // ----------------------------
    // Fetch all current data from backend
    // ----------------------------
    private async collectFrame(): Promise<void> {
        const [robots, statuses, states, stateChanges] = await Promise.all([
            Networks.fetchRobots(),
            Networks.fetchStatuses(),
            Networks.fetchStates(),
            Networks.fetchStateChanges(),
        ]);

        // Fetch neighbors for all robots in parallel
        const neighborsArray: Neighbor[] = (
            await Promise.all(robots.map((r) => Networks.fetchNeighbors(r.robot_id)))
        ).flat();

        this.patchRobots(robots);
        this.patchStatuses(statuses);
        this.patchNeighbors(neighborsArray);
        this.patchStates(states);
        this.patchStateChanges(stateChanges);

        this.updateTopology(robots, neighborsArray);
    }

    // ----------------------------
    // Patch/update functions
    // ----------------------------
    private patchRobots(robots: Robot[]): void {
        for (const r of robots) {
            const existing = this.robots.get(r.robot_id);
            this.robots.set(r.robot_id, { ...existing, ...r });
        }
    }

    private patchStatuses(statuses: Status[]): void {
        for (const s of statuses) {
            const existing = this.statuses.get(s.status_id);
            this.statuses.set(s.status_id, { ...existing, ...s });
        }
    }

    private patchNeighbors(neighbors: Neighbor[]): void {
        for (const n of neighbors) {
            const key = `${n.robot_id}-${n.neighbor}`;
            const existing = this.neighbors.get(key);
            this.neighbors.set(key, { ...existing, ...n });
        }
    }

    private patchStates(states: State[]): void {
        for (const s of states) {
            const existing = this.states.get(s.state_id);
            this.states.set(s.state_id, { ...existing, ...s });
        }
    }

    private patchStateChanges(changes: StateChange[]): void {
        for (const c of changes) {
            const existing = this.stateChanges.get(c.change_id);
            this.stateChanges.set(c.change_id, { ...existing, ...c });
        }
    }

    // ----------------------------
    // Topology update
    // ----------------------------
    private updateTopology(robots: Robot[], neighbors: Neighbor[]): void {
        const currentRobotIds = new Set(robots.map((r) => r.robot_id));

        // Update/add nodes
        for (const robot of robots) {
            const node = this.topology.get(robot.robot_id);
            if (node) {
                node.robot = robot;
                node.active = true;
            } else {
                this.topology.set(robot.robot_id, {
                    robot,
                    neighbors: new Map(),
                    active: true,
                });
            }
        }

        // Mark missing robots as inactive
        for (const [id, node] of this.topology) {
            if (!currentRobotIds.has(id)) {
                node.active = false;
            }
        }

        // Clear neighbors
        for (const node of this.topology.values()) {
            node.neighbors.clear();
        }

        // Build edges
        for (const n of neighbors) {
            const node = this.topology.get(n.robot_id);
            if (node) {
                node.neighbors.set(n.neighbor, n.strength ?? 1);
            }
        }
    }

    // ----------------------------
    // Accessors
    // ----------------------------
    public getRobots(): Robot[] {
        return Array.from(this.robots.values());
    }

    public getStatuses(): Status[] {
        return Array.from(this.statuses.values());
    }

    public getNeighbors(): Neighbor[] {
        return Array.from(this.neighbors.values());
    }

    public getStates(): State[] {
        return Array.from(this.states.values());
    }

    public getStateChanges(): StateChange[] {
        return Array.from(this.stateChanges.values());
    }

    public getTopologyNodes(): GraphNode[] {
        return Array.from(this.topology.values());
    }

    public getTopologyEdges(): { from: number; to: number; strength: number }[] {
        const edges: { from: number; to: number; strength: number }[] = [];
        for (const node of this.topology.values()) {
            for (const [neighborId, strength] of node.neighbors) {
                edges.push({ from: node.robot.robot_id, to: neighborId, strength });
            }
        }
        return edges;
    }

    public getActiveTopologyNodes(): GraphNode[] {
        return this.getTopologyNodes().filter((n) => n.active);
    }
}
