// DronesPollingService.ts
import { Robot } from "../models/Robot"
import { DroneSimulatorBackend } from "./DroneSimulatorBackend"

interface Status {
    status_id: number
    robot_id: number
    battery?: number
    cpu_1?: number
    point?: { lon: number; lat: number; alt?: number }
    orientation?: { x: number; y: number; z: number; w: number }
    last_heard: number
}

interface Neighbor {
    robot_id: number
    neighbor: number
    strength?: number
}

export class DronesPollingService {
    private baseUrl: string
    private intervalMs: number
    private timer: number | null = null
    private useSimulator: boolean
    private droneSimulatorBackend: DroneSimulatorBackend | undefined

    private addRobotsBatch: (robots: Robot[], timestamp: number) => Promise<void> | void

    constructor(options: {
        baseUrl: string
        intervalMs?: number
        addRobotsBatch: (robots: Robot[], timestamp: number) => Promise<void> | void
        useSimulator?: boolean
        droneSimulatorBackend?: DroneSimulatorBackend
    }) {
        this.baseUrl = options.baseUrl
        this.intervalMs = options.intervalMs ?? 1000
        this.addRobotsBatch = options.addRobotsBatch
        // @ts-expect-error
        this.useSimulator = options.useSimulator ?? import.meta.env.VITE_USE_SIM === 'true'
        this.droneSimulatorBackend = options.droneSimulatorBackend
    }

    // ==============================
    // START / STOP
    // ==============================
    start() {
        if (this.timer !== null) return
        this.timer = window.setInterval(() => this.pollOnce(), this.intervalMs)
        console.log(`[DronesPolling] started (${this.useSimulator ? 'SIM' : 'HTTP'})`)
    }

    stop() {
        if (this.timer !== null) {
            clearInterval(this.timer)
            this.timer = null
            console.log("[DronesPolling] stopped")
        }
    }

    // ==============================
    // MAIN POLL (BATCHED)
    // ==============================

    private async pollOnce() {
        const timestamp = Date.now()

        try {
            const robots = await this.fetchRobots()
            const mergedBatch: Robot[] = []

            for (const robot of robots) {
                try {
                    const [status, neighbors] = await Promise.all([
                        this.fetchLatestStatus(robot.robot_id),
                        this.fetchNeighbors(robot.robot_id)
                    ])

                    const connectivity: Record<string, number> = {}
                    for (const n of neighbors) {
                        const neighborRobot = robots.find(r => r.robot_id === n.neighbor)
                        if (neighborRobot) connectivity[neighborRobot.nid] = n.strength ?? 0
                    }

                    const merged: Robot = {
                        ...robot,
                        ...status,
                        connectivity
                    }

                    mergedBatch.push(merged)
                } catch (robotErr) {
                    console.warn(
                        "[DronesPolling] Failed robot fetch",
                        robot.robot_id,
                        robotErr
                    )
                }
            }

            // ✅ SINGLE BATCH WRITE
            if (mergedBatch.length) {
                await this.addRobotsBatch(mergedBatch, timestamp)
            }
        } catch (err) {
            console.error("[DronesPolling] Poll failed", err)
        }
    }

    // ==============================
    // FETCHERS (SIM OR HTTP)
    // ==============================

    private async fetchRobots(): Promise<Robot[]> {
        if (this.useSimulator && this.droneSimulatorBackend) {
            return this.droneSimulatorBackend.getRobots()
        }
        const res = await fetch(`${this.baseUrl}/robot/`)
        if (!res.ok) throw new Error("Failed to fetch robots")
        return res.json()
    }

    private async fetchLatestStatus(robotId: number): Promise<Partial<Robot>> {
        if (this.useSimulator && this.droneSimulatorBackend) {
            const statuses = this.droneSimulatorBackend.getStatus(robotId)
            if (!statuses.length) return {}
            const latest = statuses[statuses.length - 1]
            return {
                battery: latest.battery,
                cpu_1: latest.cpu_1,
                point: latest.point,
                orientation: latest.orientation,
                last_heard: latest.last_heard
            }
        }

        const res = await fetch(`${this.baseUrl}/status/robot/${robotId}`)
        if (!res.ok) return {}

        const list: Status[] = await res.json()
        if (!list.length) return {}

        const latest = list[list.length - 1]

        return {
            battery: latest.battery,
            cpu_1: latest.cpu_1,
            point: latest.point,
            orientation: latest.orientation,
            last_heard: latest.last_heard
        }
    }

    private async fetchNeighbors(robotId: number): Promise<Neighbor[]> {
        if (this.useSimulator && this.droneSimulatorBackend) {
            return this.droneSimulatorBackend.getNeighbors(robotId)
        }
        const res = await fetch(`${this.baseUrl}/neighbor/robot/${robotId}`)
        if (!res.ok) return []
        return res.json()
    }
}
