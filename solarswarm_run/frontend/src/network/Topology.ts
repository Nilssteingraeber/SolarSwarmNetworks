// Topology.ts
import type { Robot, Neighbor } from "./types";

/**
 * GraphNode represents a robot in the topology
 */
export interface GraphNode {
    robot: Robot;
    neighbors: Map<number, number>; // neighbor robot_id -> strength
    active: boolean; // true if robot is currently present
}

/**
 * Topology class
 * Builds and updates a robot network graph
 */
export class Topology {
    private nodes: Map<number, GraphNode> = new Map();

    constructor() { }

    /**
     * Update topology from a list of robots and neighbors
     */
    public update(robots: Robot[], neighbors: Neighbor[]): void {
        const currentRobotIds = new Set(robots.map((r) => r.robot_id));

        // Update or add nodes
        for (const robot of robots) {
            const node = this.nodes.get(robot.robot_id);
            if (node) {
                node.robot = robot;
                node.active = true;
            } else {
                this.nodes.set(robot.robot_id, {
                    robot,
                    neighbors: new Map(),
                    active: true,
                });
            }
        }

        // Mark missing robots as inactive
        for (const [id, node] of this.nodes) {
            if (!currentRobotIds.has(id)) {
                node.active = false;
            }
        }

        // Clear previous neighbors
        for (const node of this.nodes.values()) {
            node.neighbors.clear();
        }

        // Build edges
        for (const neighbor of neighbors) {
            const node = this.nodes.get(neighbor.robot_id);
            if (node) {
                node.neighbors.set(neighbor.neighbor, neighbor.strength ?? 1);
            }
        }
    }

    /**
     * Get all nodes
     */
    public getNodes(): GraphNode[] {
        return Array.from(this.nodes.values());
    }

    /**
     * Get a list of edges
     */
    public getEdges(): { from: number; to: number; strength: number }[] {
        const edges: { from: number; to: number; strength: number }[] = [];
        for (const node of this.nodes.values()) {
            for (const [neighborId, strength] of node.neighbors) {
                edges.push({ from: node.robot.robot_id, to: neighborId, strength });
            }
        }
        return edges;
    }

    /**
     * Get active robots only
     */
    public getActiveRobots(): GraphNode[] {
        return this.getNodes().filter((n) => n.active);
    }
}
