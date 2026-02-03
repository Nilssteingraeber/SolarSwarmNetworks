// DronesPollingService.ts
import { Robot } from "../models/Robot"
import { useServiceInterfaceStore } from "../stores/ServiceInterfaceStore"
import { DroneSimulatorBackend } from "./DroneSimulatorBackend"
import { parseInterface, prettyPrintInterface } from "./NodeRosInterfaces"

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
        addRobotsBatch: (robots: Robot[], timestamp: number) => Promise<void> | void
        useSimulator?: boolean
        droneSimulatorBackend?: DroneSimulatorBackend
    }) {
        this.baseUrl = options.baseUrl
        this.intervalMs = options.intervalMs ?? 1000
        this.addRobotsBatch = options.addRobotsBatch
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
            // 1️⃣ Fetch all robots
            const robots = await this.fetchRobots()
            if (!robots.length) return

            // Map robots by NID for quick neighbor lookup
            const robotById = new Map<number, Robot>()
            robots.forEach(r => robotById.set(r.robot_id, r))

            // 2️⃣ Prepare array of IDs for batch queries
            const robotIds = robots.map(r => r.robot_id)

            // 3️⃣ Fetch all statuses and neighbors in parallel
            const [statusesResponse, neighborsResponse] = await Promise.all([
                this.fetchLatestStatusBatch(robotIds),      // expects { [robotId]: Status }
                this.fetchNeighborsBatch(robotIds)          // expects { [robotId]: Neighbor[] }
            ])

            // 4️⃣ Merge data into a single array of Robots
            const mergedBatch: Robot[] = robots.map(r => {
                const status = statusesResponse[r.robot_id] ?? {}
                const neighborList = neighborsResponse[r.robot_id] ?? []

                const connectivity: Record<string, number> = {}
                for (const n of neighborList) {
                    const neighborRobot = robotById.get(n.neighbor)
                    if (neighborRobot) connectivity[neighborRobot.nid] = n.strength ?? 0
                }

                return {
                    ...r,
                    ...status,
                    connectivity
                } as Robot
            })

            // 5️⃣ Push the batch to your store
            if (mergedBatch.length) await this.addRobotsBatch(mergedBatch, timestamp)

        } catch (err) {
            console.error("[DronesPolling] Poll failed", err)
        }
    }




    // ==============================
    // FETCHERS (SIM OR HTTP)
    // ==============================

    private async fetchRobots(): Promise<Robot[]> {
        try {
            if (this.useSimulator) {
                return this.droneSimulatorBackend
                    ? await this.droneSimulatorBackend.getRobots()
                    : []
            }

            const controller = new AbortController()
            setTimeout(() => controller.abort(), 5000)

            const res = await fetch(`${this.baseUrl}/robot/`, {
                signal: controller.signal,
            })

            if (!res.ok) throw new Error(`HTTP ${res.status}`)

            const data = await res.json()

            return Array.isArray(data) ? data : []
        } catch (err) {
            console.error("[fetchRobots] failed:", err)
            return []
        }
    }

    public async fetchServicesList(nid: string) {
        const res = await fetch(`${this.baseUrl}/ros/${nid}/services`)
        const json = (await res.json())?.services as { name: string, type: string }[]

        console.log(json)

        const robotServiceInterface = json.find(
            s => s.type === "custom_interfaces/srv/RobotInterfaceInfo"
        )

        if (!robotServiceInterface) {
            console.error("RobotInterfaceInfo service not found")
            return []
        }

        const customServices = json.filter(
            s => {

                return s.type.startsWith("custom_interfaces/srv/") && s.name.indexOf(nid) !== -1
            }
        )

        console.log(customServices)

        const service_info = await Promise.all(
            customServices.map(async (service) => {

                const res = await fetch(`${this.baseUrl}/ros/${nid}/service_info`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        info_service_name: robotServiceInterface.name,
                        target_interface: service.type,
                    }),
                })

                const json = await res.json()

                return {
                    serviceName: service.name,
                    serviceType: service.type,
                    parsed: parseInterface(json.definition),
                }
            })
        )

        service_info.forEach((svc) => {
            this.interfaceStore.addOrUpdate(nid, svc)
            console.debug(prettyPrintInterface(svc))
        })

        return service_info
    }



    private async fetchLatestStatus(robotId: number): Promise<Partial<Robot>> {
        if (this.useSimulator && this.droneSimulatorBackend) {
            const statuses = this.droneSimulatorBackend.getStatus(robotId)
            if (!statuses.length) return {}
            const latest = statuses.pop()

            return {
                battery: latest?.battery,
                cpu_1: latest?.cpu_1,
                point: latest?.point,
                orientation: latest?.orientation,
                last_heard: latest?.last_heard
            }
        }

        const res = await fetch(`${this.baseUrl}/status/robot/${robotId}`)

        if (!res.ok) { console.error("Err: " + res.status + "_" + res.statusText); return {} }

        const latest: Status = await res.json()
        if (!latest) return {}

        return {
            battery: latest?.battery,
            cpu_1: latest?.cpu_1,
            point: { lat: latest?.point?.x, lon: latest.point?.y, alt: 125.0 },
            orientation: latest?.orientation,
            last_heard: new Date().getTime()
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



    private async fetchLatestStatusBatch(robotIds: number[]): Promise<Record<number, Partial<Robot>>> {
        if (this.useSimulator && this.droneSimulatorBackend) {
            const result: Record<number, Partial<Robot>> = {}
            for (const id of robotIds) {
                const statuses = this.droneSimulatorBackend.getStatus(id)
                if (!statuses.length) continue
                const latest = statuses[statuses.length - 1]
                result[id] = {
                    battery: latest?.battery,
                    cpu_1: latest?.cpu_1,
                    point: latest?.point,
                    orientation: latest?.orientation,
                    last_heard: latest?.last_heard
                }
            }
            return result
        }


        const res = await fetch(`${this.baseUrl}/status/robots`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ robot_ids: robotIds }),
        })

        if (!res.ok) {
            console.error("fetchLatestStatusBatch failed:", res.status, res.statusText)
            return {}
        }

        const data: Status[] = await res.json()
        const result: Record<number, Partial<Robot>> = {}

        for (const s of data) {

            result[s.robot_id] = {
                battery: s.battery,
                cpu_1: s.cpu_1,
                point: { lat: s.point?.x, lon: s.point?.y, alt: 125.0 },
                orientation: s.orientation,
                last_heard: new Date().getTime()
            }
        }
        return result
    }

    private async fetchNeighborsBatch(robotIds: number[]): Promise<Record<number, Neighbor[]>> {
        if (this.useSimulator && this.droneSimulatorBackend) {
            const result: Record<number, Neighbor[]> = {}
            for (const id of robotIds) {
                result[id] = this.droneSimulatorBackend.getNeighbors(id)
            }
            return result
        }

        try {
            const res = await fetch(`${this.baseUrl}/neighbor/robots`, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ robot_ids: robotIds }),
            })

            if (!res.ok) {
                console.error("fetchNeighborsBatch failed:", res.status, res.statusText)
                return {}
            }

            const data: Record<number, Neighbor[]> = await res.json()
            return data
        } catch (err) {
            console.error("fetchNeighborsBatch error:", err)
            return {}
        }
    }


}
