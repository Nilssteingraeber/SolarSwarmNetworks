import { ref, computed, watch } from 'vue'
import { defineStore } from 'pinia'
import { useDroneHistoryStore } from '../dronesData/DroneHistoryStore'
import { useTimeStore } from './TimeStore'
import { DronesPollingService } from '../dronesData/DronesPollingService'
import type { Robot } from '../models/Robot'
import { useDroneEntityStore } from '../dronesData/DroneEntityStore'

/**
 * simpleThrottle: Vanilla implementation to limit function execution rate.
 * Ensures the function is called at most once every 'limit' milliseconds.
 */
function simpleThrottle<T extends (...args: any[]) => any>(func: T, limit: number): (...args: Parameters<T>) => void {
    let inThrottle: boolean = false;
    let lastFunc: ReturnType<typeof setTimeout>;
    let lastRan: number;

    return function (this: any, ...args: Parameters<T>) {
        if (!inThrottle) {
            func.apply(this, args);
            lastRan = Date.now();
            inThrottle = true;

            // Set a timeout to clear the throttle flag after the limit
            setTimeout(() => inThrottle = false, limit);
        } else {
            // Optional: Ensure the last call always runs (trailing edge)
            clearTimeout(lastFunc);
            lastFunc = setTimeout(() => {
                if ((Date.now() - lastRan) >= limit) {
                    func.apply(this, args);
                    lastRan = Date.now();
                }
            }, limit - (Date.now() - lastRan));
        }
    };
}

export const UseViewedDroneStore = defineStore('currentlyViewedDrone', () => {
    // --- STORES ---
    const historyStore = useDroneHistoryStore()
    const timeStore = useTimeStore()
    const entityStore = useDroneEntityStore()

    // --- STATE ---
    const viewedNid = ref<string | undefined>(undefined)
    const viewedRobot = ref<Robot | undefined>(undefined)
    const dronesPollingService = ref<DronesPollingService | undefined>(undefined)

    // --- GETTERS (Computed) ---
    const isSelected = computed(() => !!viewedNid.value)

    // --- INTERNAL UTILS ---

    /**
     * Throttled Flight Path Drawer.
     * Problem: Cesium Primitives are heavy to rebuild.
     * Solution: We limit this function to run at most once every 100ms (10 FPS).
     */
    const throttledDraw = simpleThrottle((nid: string, time: number) => {
        // Ensure we are still viewing the same drone before drawing
        if (viewedNid.value === nid) {
            entityStore.drawFlightPath(nid, time)
        }
    }, 100) // 100ms delay

    // --- ACTIONS ---

    /**
     * Sets the currently focused drone.
     * Cleans up the flight path of the previous drone if needed.
     */
    const setDrone = (robot: Robot | null, nid?: string) => {
        const targetNid = nid ?? robot?.nid

        // If switching to a different drone (or clearing), remove the old path
        if (viewedNid.value && viewedNid.value !== targetNid) {
            entityStore.removeFlightPathLines(viewedNid.value)
        }

        viewedNid.value = targetNid
        viewedRobot.value = robot ?? undefined

        // If we selected a new drone, draw the path immediately (force update)
        if (targetNid) {
            updateViewedRobot()
        }
    }

    /**
     * Updates the viewed drone's data and 3D visualization based on the current time.
     */
    const updateViewedRobot = async (): Promise<boolean> => {
        if (!viewedNid.value) {
            viewedRobot.value = undefined
            return false
        }

        const currentTime = timeStore.currentTime

        // 1. Fetch Snapshot for UI (Text/Sidebar)
        // We use the central history store to get the data state at this time
        const snapshot = await historyStore.getSnapshotAt(
            viewedNid.value,
            currentTime,
            15_000 // 15s tolerance window
        )

        // 2. Update 3D Flight Path (Throttled)
        // We pass the ID and Time to the throttled function.
        // It will drop calls if they happen too frequently.
        throttledDraw(viewedNid.value, currentTime)

        viewedRobot.value = snapshot?.data ?? undefined
        return !!snapshot
    }

    /**
     * Pure Utility: Fetches a snapshot without changing the active viewed state.
     */
    const getDroneAt = async (timestamp: number): Promise<Robot | undefined> => {
        if (!viewedNid.value) return undefined
        const entry = await historyStore.getSnapshotAt(viewedNid.value, timestamp, 15_000)
        return entry?.data
    }

    const clear = () => {
        if (viewedNid.value) {
            entityStore.removeFlightPathLines(viewedNid.value)
        }
        viewedNid.value = undefined
        viewedRobot.value = undefined
    }

    const setDronesPollingService = (service: DronesPollingService) => {
        dronesPollingService.value = service
    }

    // --- REACTIVE LOGIC (Watcher) ---
    // Watch for time changes to keep the UI and Path in sync
    watch(() => timeStore.currentTime, () => {
        if (isSelected.value) {
            updateViewedRobot()
        }
    })

    return {
        // State
        viewedNid,
        viewedRobot,
        dronesPollingService,

        // Getters
        isSelected,

        // Actions
        setDrone,
        updateViewedRobot,
        getDroneAt,
        clear,
        setDronesPollingService
    }
})