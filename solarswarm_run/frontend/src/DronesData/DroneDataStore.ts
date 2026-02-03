import { defineStore } from 'pinia'
import type { Robot } from '../models/Robot'

export interface DroneRecord {
    robot: Robot
    last_heard: number
}

export const useDroneDataStore = defineStore('droneData', {
    state: () => ({
        drones: [] as DroneRecord[],
        currentlyViewed: undefined as Robot | undefined
    }),

    getters: {
        byId: (state) => (nid: string) =>
            state.drones.find(d => d.robot.nid === nid),

        all: (state) => () =>
            state.drones
    },

    actions: {
        setCurrentlyViewed(robot: Robot | undefined) {
            this.currentlyViewed = robot
        },

        upsertDrone(robot: Partial<Robot> & { nid: string }) {

            // Normalize neighbors array → connectivity map
            if ((robot as any).neighbors && !robot.connectivity) {
                const map: Record<string, number> = {}
                for (const n of (robot as any).neighbors) {
                    if (n?.nid) map[n.nid] = n.strength ?? 0
                }
                ; (robot as any).connectivity = map
            }

            const existing = this.drones.find(d => d.robot.nid === robot.nid)

            if (existing) {
                Object.assign(existing.robot, robot)
                return
            }

            const newRobot = robot as Robot
            this.drones.push({
                robot: newRobot,
                last_heard: newRobot.last_heard || 0
            })

        },

        removeDrone(nid: string) {
            this.drones = this.drones.filter(d => d.robot.nid !== nid)
            if (this.currentlyViewed?.nid === nid) {
                this.currentlyViewed = undefined
            }
        },

        clear() {
            this.drones = []
            this.currentlyViewed = undefined
        }
    }
})
