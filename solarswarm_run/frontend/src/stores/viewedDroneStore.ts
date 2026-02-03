import { ref, Ref } from 'vue'
import { defineStore } from 'pinia'
import { useDroneHistoryStore } from '../DronesData/DroneHistoryStore'
import type { Robot } from '../models/Robot'
import { useTimeStore } from './TimeStore'
import { useDroneEntityStore } from '../DronesData/DroneEntityStore'
import { DronesPollingService } from '../DronesData/DronesPollingService'

/**
 * Store for managing the currently viewed drone.
 * Tracks which drone is selected and provides convenient methods
 * to fetch its historical or current state.
 */
export const UseViewedDroneStore = defineStore('currentlyViewedDrone', () => {
    // ---------------------------
    // STATE
    // ---------------------------
    /** Currently viewed drone NID (unique identifier) */
    const viewedNid: Ref<string | undefined> = ref(undefined)

    /** Currently viewed drone snapshot (latest fetched from history) */
    const viewedRobot: Ref<Robot | undefined> = ref(undefined)

    const dronesPollingService: Ref<DronesPollingService> = ref(undefined)

    // ---------------------------
    // DEPENDENT STORES
    // ---------------------------
    const history = useDroneHistoryStore()
    const timeStore = useTimeStore()

    // ---------------------------
    // ACTIONS
    // ---------------------------

    /**
     * Set the currently viewed drone by Robot object or NID.
     * If robot is null, clears the current selection.
     */
    const setDrone = (robot: Robot | null, nid?: string) => {
        viewedNid.value = nid ?? robot?.nid
        if (!robot) viewedRobot.value = undefined
    }

    /**
     * Fetch and update the `viewedRobot` from history at the current time.
     * Returns true if a snapshot was found, false otherwise.
     */
    const updateViewedRobot = async (): Promise<boolean> => {

        if (!viewedNid.value) {
            viewedRobot.value = undefined
            return false
        }

        const snapshot = await history.getSnapshotAt(
            viewedNid.value,
            timeStore.currentTime,
            15_000
        )

        useDroneEntityStore().drawFlightPath(viewedNid.value, useTimeStore().currentTime, 500)
        viewedRobot.value = snapshot ?? undefined
        return !!snapshot
    }

    /**
     * Get a fresh snapshot of the currently viewed drone without modifying the store.
     * Useful for temporary calculations or previews.
     */
    const getDrone = async (): Promise<Robot | undefined> => {
        if (!viewedNid.value) return undefined
        return history.getSnapshotAt(
            viewedNid.value,
            timeStore.currentTime,
            15_000
        )
    }

    /**
     * Get a snapshot of the currently viewed drone at a specific timestamp.
     * Does not modify the stored `viewedRobot`.
     */
    const getDroneAt = async (timestamp: number): Promise<Robot | undefined> => {
        if (!viewedNid.value) return undefined
        return history.getSnapshotAt(viewedNid.value, timestamp, 15_000)
    }

    /**
     * Return the currently cached `viewedRobot`.
     * This is the last value fetched by `updateViewedRobot()`.
     */
    const getViewedRobot = (): Robot | undefined => viewedRobot.value

    /**
     * Clears the current viewed drone selection and cached snapshot.
     */
    const clear = () => {
        viewedNid.value = undefined
        viewedRobot.value = undefined
    }

    const setDronesPollingService = (s: DronesPollingService) => {
        dronesPollingService.value = s
    }
    // ---------------------------
    // RETURNED OBJECT
    // ---------------------------
    return {
        // state
        viewedNid,
        viewedRobot,
        dronesPollingService,

        setDronesPollingService,

        // actions
        setDrone,
        updateViewedRobot,
        getDrone,
        getDroneAt,
        getViewedRobot,
        clear,
    }
})
