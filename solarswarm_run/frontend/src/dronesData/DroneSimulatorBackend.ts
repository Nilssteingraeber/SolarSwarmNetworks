import type { Robot, RobotLocation } from '../models/Robot'

// ------------------------------------------
// API MODELS (MATCH POLLING SERVICE)
// ------------------------------------------

interface Status {
    status_id: number
    robot_id: number
    battery?: number
    cpu_1?: number
    point?: RobotLocation | Record<string, any>;
    orientation?: { x?: number; y?: number; z?: number; w?: number }
    last_heard: number
}

interface Neighbor {
    robot_id: number
    neighbor: number
    strength?: number
}

// ------------------------------------------

interface SimulatedRobot extends Robot {
    velocity: { lat: number; lon: number; alt: number }
    isReporting: boolean
    heading: number
    offlineUntil?: number
    permanentlyFailed?: boolean
}

// ------------------------------------------

export class DroneSimulatorBackend {
    private robots: SimulatedRobot[] = []
    private statusTable = new Map<number, Status[]>()
    private nextStatusId = 1

    // --- GEO ---
    private bochumLat = 51.445989
    private bochumLon = 7.274733

    // --- PHYSICS ---
    private speedFactor = 0.00005
    private maxAltitude = 250

    // --- FAILURES ---
    private outageChance = 0.0
    private outageDuration = 20_000
    private permanentFailureChance = 0.000
    private reportingFailureChance = 0.00

    // --- CONNECTIVITY ---
    private maxConnectivityRangeKm = 0.125
    private distanceToStrengthFactor = 0.8

    // --- GROWTH ---
    private nextDroneId = 1
    private nextGrowthTime = Date.now() + 60_000
    private growthInterval = 60_000

    constructor(count = 5) {
        this.generateDrones(count)
    }

    // =====================================================
    // PUBLIC API
    // =====================================================

    /** GET /robot/ */
    getRobots(): Robot[] {
        return this.robots.map(r => ({
            robot_id: r.robot_id,
            nid: r.nid,
            state_id: r.state_id,
            display_name: r.display_name,
            ipv4: r.ipv4
        }))
    }

    /** GET /status/robot/:id */
    getStatus(robotId: number): Status[] {
        return this.statusTable.get(robotId) ?? []
    }

    /** GET /neighbor/robot/:id */
    getNeighbors(robotId: number): Neighbor[] {
        const source = this.robots.find(r => r.robot_id === robotId)
        if (!source?.connectivity) return []

        return Object.entries(source.connectivity).map(([nid, strength]) => {
            const neighbor = this.robots.find(d => d.nid === nid)
            return {
                robot_id: robotId,
                neighbor: neighbor?.robot_id ?? -1,
                strength
            }
        }).filter(n => n.neighbor !== -1)
    }

    // =====================================================
    // TICK LOOP (CALL EVERY SECOND)
    // =====================================================

    tick() {
        const now = Date.now()

        // --- AUTO GROW ---
        if (now >= this.nextGrowthTime) {
            this.addOneDrone()
            this.nextGrowthTime = now + this.growthInterval
        }

        // --- UPDATE DRONES ---
        for (const drone of this.robots) {
            if (drone.permanentlyFailed) continue

            if (!drone.offlineUntil && Math.random() < this.outageChance) {
                drone.offlineUntil = now + this.outageDuration
            }

            const isOffline = drone.offlineUntil && now < drone.offlineUntil
            drone.isReporting = !isOffline && Math.random() > this.reportingFailureChance

            if (!isOffline) {
                // --- MOVEMENT ---
                if (Math.random() < 0.075) {
                    drone.heading += (Math.random() - 0.5) * Math.PI / 4
                    drone.velocity.lat = Math.cos(drone.heading) * this.speedFactor
                    drone.velocity.lon = Math.sin(drone.heading) * this.speedFactor
                    drone.velocity.alt += (Math.random() - 0.5) * 0.001
                }

                drone.point = {
                    lat: drone.point!.lat + drone.velocity.lat,
                    lon: drone.point!.lon + drone.velocity.lon,
                    alt: Math.min(this.maxAltitude, Math.max(20, drone.point!.alt! + drone.velocity.alt * 10))
                }

                drone.battery = Math.max(0, drone.battery! - Math.random() * 0.7)
                drone.cpu_1 = Math.min(100, Math.max(0, drone.cpu_1! + (Math.random() - 0.5) * 16))
            }

            // --- STORE STATUS ROW ---
            if (drone.isReporting) {
                drone.last_heard = now

                const row: Status = {
                    status_id: this.nextStatusId++,
                    robot_id: drone.robot_id,
                    battery: drone.battery,
                    cpu_1: drone.cpu_1,
                    point: { ...drone.point! },
                    orientation: { ...drone.orientation! },
                    last_heard: now
                }

                if (!this.statusTable.has(drone.robot_id)) {
                    this.statusTable.set(drone.robot_id, [])
                }

                this.statusTable.get(drone.robot_id)!.push(row)
            }
        }

        this.updateConnectivity()
    }

    // =====================================================
    // CONNECTIVITY PHYSICS
    // =====================================================

    private updateConnectivity() {
        const active = this.robots.filter(d => d.isReporting && !d.permanentlyFailed)
        active.forEach(d => d.connectivity = {})

        for (let i = 0; i < active.length; i++) {
            for (let j = i + 1; j < active.length; j++) {
                const A = active[i]
                const B = active[j]
                const dist = this.calculateDistance(A.point!, B.point!)

                if (dist <= this.maxConnectivityRangeKm) {
                    let strength = 1.0 - (dist / this.maxConnectivityRangeKm)
                    strength = Math.pow(strength, this.distanceToStrengthFactor)

                    A.connectivity![B.nid] = strength
                    B.connectivity![A.nid] = strength
                }
            }
        }
    }

    private calculateDistance(p1: RobotLocation, p2: RobotLocation): number {
        const R = 6371
        const dLat = (p2.lat - p1.lat) * Math.PI / 180
        const dLon = (p2.lon - p1.lon) * Math.PI / 180

        const a =
            Math.sin(dLat / 2) ** 2 +
            Math.cos(p1.lat * Math.PI / 180) *
            Math.cos(p2.lat * Math.PI / 180) *
            Math.sin(dLon / 2) ** 2

        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))
        const dAlt = (p2.alt! - p1.alt!) / 1000
        return Math.sqrt((R * c) ** 2 + dAlt ** 2)
    }

    // =====================================================
    // SPAWN
    // =====================================================

    private generateDrones(count: number) {
        for (let i = 0; i < count; i++) this.addOneDrone()
    }

    private addOneDrone() {
        const id = this.nextDroneId++
        const heading = Math.random() * 2 * Math.PI

        const drone: SimulatedRobot = {
            robot_id: id,
            nid: `drone-${id}`,
            state_id: 1,
            display_name: `Drone ${id}`,
            ipv4: `192.168.0.${id + 10}`,
            battery: 100,
            cpu_1: Math.random() * 50,
            point: {
                lat: this.bochumLat + (Math.random() - 0.5) * 0.01,
                lon: this.bochumLon + (Math.random() - 0.5) * 0.01,
                alt: 180
            },
            orientation: {
                x: 0,
                y: 0,
                z: 0,
                w: 0
            },
            last_heard: Date.now(),
            connectivity: {},

            velocity: {
                lat: Math.cos(heading) * this.speedFactor,
                lon: Math.sin(heading) * this.speedFactor,
                alt: 0
            },
            isReporting: true,
            heading
        }

        this.robots.push(drone)
    }
}
