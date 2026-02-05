import { Robot } from "../models/Robot"
import { useServiceInterfaceStore } from "../stores/ServiceInterfaceStore"
import { useDroneEntityStore } from "./DroneEntityStore"
import { DroneSimulatorBackend } from "./DroneSimulatorBackend"
import { NamedParsedInterface } from "./NodeRosInterfaces"

interface Status {
    status_id: number
    robot_id: number
    battery?: number
    cpu_1?: number
    point?: Robot["point"]
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
    private interfaceStore = useServiceInterfaceStore()

    constructor(options: {
        baseUrl: string
        intervalMs?: number
        intervalRarelyMs?: number
        addRobotsBatch: (robots: Robot[], timestamp: number) => Promise<void> | void
        useSimulator?: boolean
        droneSimulatorBackend?: DroneSimulatorBackend
    }) {
        this.baseUrl = options.baseUrl
        this.intervalMs = options.intervalMs ?? 1000
        this.intervalMs = options.intervalRarelyMs ?? 15000
        this.addRobotsBatch = options.addRobotsBatch
        this.useSimulator = options.useSimulator ?? import.meta.env.VITE_USE_SIM === 'true'
        this.droneSimulatorBackend = options.droneSimulatorBackend
    }

    start() {
        if (this.timer !== null) return
        this.timer = window.setInterval(() => this.pollOnce(), this.intervalMs)
        this.timer = window.setInterval(() => this.pollRarely(), this.intervalMs)
        console.log(`[DronesPolling] started (${this.useSimulator ? 'SIM' : 'HTTP'})`)

        this.pollRarely()
        this.pollOnce()
    }

    stop() {
        if (this.timer !== null) {
            clearInterval(this.timer)
            this.timer = null
            console.log("[DronesPolling] stopped")
        }
    }

    private async pollRarely() {
        useDroneEntityStore().setStateNames(await this.fetchStateNames())
    }

    private async pollOnce() {
        const timestamp = Date.now()
        try {
            const robots = await this.fetchRobots()
            if (!robots.length) return

            const robotById = new Map<number, Robot>()
            robots.forEach(r => robotById.set(r.robot_id, r))
            const robotIds = robots.map(r => r.robot_id)

            const [statusesResponse, neighborsResponse] = await Promise.all([
                this.fetchLatestStatusBatch(robotIds),
                this.fetchNeighborsBatch(robotIds)
            ])

            const mergedBatch: Robot[] = robots.map(r => {
                const status = statusesResponse[r.robot_id] ?? {}
                const neighborList = neighborsResponse[r.robot_id] ?? []
                const connectivity: Record<string, number> = {}
                for (const n of neighborList) {
                    const neighborRobot = robotById.get(n.neighbor)
                    if (neighborRobot) connectivity[neighborRobot.nid] = n.strength ?? 0
                }

                return { ...r, ...status, connectivity } as Robot
            })

            if (mergedBatch.length) await this.addRobotsBatch(mergedBatch, timestamp)
        } catch (err) {
            console.error("[DronesPolling] Poll failed", err)
        }
    }

    private async fetchStateNames(): Promise<Record<number, string>> {
        try {
            const res = await fetch(`${this.baseUrl}/states`)
            if (!res.ok) throw new Error(`HTTP ${res.status}`)
            return await res.json()
        } catch {
            return {}
        }
    }

    private async fetchRobots(): Promise<Robot[]> {
        try {
            if (this.useSimulator) return this.droneSimulatorBackend?.getRobots() ?? []
            const res = await fetch(`${this.baseUrl}/robot`)
            if (!res.ok) throw new Error(`HTTP ${res.status}`)
            return await res.json()
        } catch (err) {
            return []
        }
    }

    /**
     * ALIGNED: Fetches the enriched service list directly from the backend.
     * The backend handles filtering and interface parsing.
     */
    public async fetchServicesList(nid: string): Promise<NamedParsedInterface[]> {
        try {
            const res = await fetch(`${this.baseUrl}/robot/${nid}/services`)
            if (!res.ok) throw new Error(`HTTP ${res.status}`)

            const enrichedServices: NamedParsedInterface[] = await res.json()

            enrichedServices.forEach(svc => {
                this.interfaceStore.addOrUpdate(nid, svc)
            })

            return enrichedServices
        } catch (err) {
            console.error("[fetchServicesList] failed:", err)
            return []
        }
    }

    public async callRosService(
        nid: string,
        serviceName: string,
        serviceType: string,
        args: Record<string, any>
    ): Promise<any> {
        const payload = { service_name: serviceName, service_type: serviceType, arguments: args };
        const response = await fetch(`${this.baseUrl}/robot/${nid}/call_service`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        });
        if (!response.ok) throw new Error(`Service call failed`);
        return await response.json();
    }

    private async fetchLatestStatusBatch(robotIds: number[]): Promise<Record<number, Partial<Robot>>> {
        const res = await fetch(`${this.baseUrl}/status/robots`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ robot_ids: robotIds }),
        })
        const data: Status[] = await res.json()
        const result: Record<number, Partial<Robot>> = {}
        for (const s of data) {
            result[s.robot_id] = {
                battery: s.battery,
                cpu_1: s.cpu_1,
                point: { lat: s.point?.x, lon: s.point?.y, alt: 125.0 },
                orientation: s.orientation,
                last_heard: s.last_heard
            }
        }
        return result
    }

    private async fetchNeighborsBatch(robotIds: number[]): Promise<Record<number, Neighbor[]>> {
        const res = await fetch(`${this.baseUrl}/neighbor/robots`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ robot_ids: robotIds }),
        })
        return await res.json()
    }
}