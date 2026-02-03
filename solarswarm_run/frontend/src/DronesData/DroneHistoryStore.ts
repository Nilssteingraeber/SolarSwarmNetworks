import { ref, type Ref, shallowRef, triggerRef, reactive } from 'vue'
import { defineStore } from 'pinia'
import type { Robot } from '../models/Robot'
import { useTimeStore } from '../stores/TimeStore'

export interface RobotHistoryEntry {
    timestamp: number
    nid: string
    data: Robot
}

// --- HIGH PERFORMANCE UTILITIES ---

function findClosestIndex(history: RobotHistoryEntry[], target: number): number {
    if (history.length === 0) return -1
    if (target <= history[0].timestamp) return 0
    if (target >= history[history.length - 1].timestamp) return history.length - 1

    let left = 0, right = history.length - 1
    while (left <= right) {
        const mid = (left + right) >>> 1
        const val = history[mid].timestamp
        if (val === target) return mid
        else if (val < target) left = mid + 1
        else right = mid - 1
    }
    const idx1 = left < history.length ? left : left - 1
    const idx2 = left - 1 >= 0 ? left - 1 : 0
    return Math.abs(history[idx1].timestamp - target) < Math.abs(history[idx2].timestamp - target) ? idx1 : idx2
}

function findInsertionIndex(history: RobotHistoryEntry[], timestamp: number): number {
    let left = 0, right = history.length - 1
    while (left <= right) {
        const mid = (left + right) >>> 1
        if (history[mid].timestamp === timestamp) return -1
        if (history[mid].timestamp < timestamp) left = mid + 1
        else right = mid - 1
    }
    return left
}

export const useDroneHistoryStore = defineStore('droneHistory', () => {
    // 1. Worker for WRITES
    const worker = new Worker(
        new URL('../workers/db.worker.ts', import.meta.url),
        { type: 'module' }
    );

    const maxEntriesPerDrone = ref(5000)
    const cache = shallowRef(new Map<string, RobotHistoryEntry[]>())
    const activeRobotCount = ref(0)

    // Metrics
    const isLoading = ref(false)
    const dbLoadCount = ref(0)

    // Known NIDs
    const knownNids = reactive(new Set<string>())

    // --- CACHE MANAGEMENT ---
    const insertIntoCacheOptimized = (history: RobotHistoryEntry[], entry: RobotHistoryEntry) => {
        const len = history.length;
        if (len === 0 || entry.timestamp >= history[len - 1].timestamp) {
            history.push(entry);
            return;
        }
        const idx = findInsertionIndex(history, entry.timestamp);
        if (idx !== -1) history.splice(idx, 0, entry);
    }

    const addRobotsBatch = (robots: Robot[], timestamp?: number) => {
        if (!robots.length) return
        const ts = timestamp ?? Date.now()
        activeRobotCount.value = robots.length

        const dbEntries = new Array(robots.length)
        const map = cache.value

        for (let i = 0; i < robots.length; i++) {
            const r = robots[i]
            const entry = { nid: r.nid, timestamp: ts, data: r }
            dbEntries[i] = entry

            if (!knownNids.has(r.nid)) knownNids.add(r.nid)

            let history = map.get(r.nid)
            if (!history) {
                history = []
                map.set(r.nid, history)
            }
            insertIntoCacheOptimized(history, entry)
        }

        triggerRef(cache)
        worker.postMessage({ type: 'WRITE_BATCH', payload: dbEntries })
        schedulePrune()
    }

    // --- PRUNING ---
    let pruneTimeout: number | null = null
    const schedulePrune = () => {
        if (pruneTimeout) return
        pruneTimeout = window.setTimeout(() => {
            pruneAllCaches()
            pruneTimeout = null
        }, 5000)
    }

    const pruneAllCaches = () => {
        const map = cache.value
        const max = maxEntriesPerDrone.value
        for (const history of map.values()) {
            if (history.length > max + 500) {
                const startIdx = history.length - max
                if (startIdx > 0) history.splice(0, startIdx)
            }
        }
    }

    // --- DB INIT ---
    let _db: IDBDatabase | null = null
    const initDBRead = async (): Promise<IDBDatabase> => {
        if (_db) return _db;
        return new Promise((resolve, reject) => {
            // CRITICAL: Version 2
            const req = indexedDB.open('DroneHistoryDB', 2);

            req.onupgradeneeded = (e: any) => {
                const db = e.target.result;
                // CRITICAL: Ensure store name is 'history'
                if (!db.objectStoreNames.contains('history')) {
                    const store = db.createObjectStore('history', { keyPath: ['nid', 'timestamp'] });
                    store.createIndex('nid_timestamp', ['nid', 'timestamp']);
                }
            };

            req.onsuccess = () => { _db = req.result; resolve(_db!) }
            req.onerror = (e) => reject(e)
        })
    }

    // --- ACTIONS ---
    const discoverAvailableDrones = async () => {
        const db = await initDBRead()
        return new Promise<void>((resolve) => {
            isLoading.value = true
            const tx = db.transaction('history', 'readonly')
            const store = tx.objectStore('history')
            const req = store.openCursor()
            const seen = new Set<string>()

            req.onsuccess = (e: any) => {
                const cursor = e.target.result
                if (cursor) {
                    const nid = cursor.value.nid
                    if (nid && !seen.has(nid)) {
                        seen.add(nid)
                        knownNids.add(nid)
                    }
                    cursor.continue()
                } else {
                    isLoading.value = false
                    resolve()
                }
            }
            req.onerror = () => { isLoading.value = false; resolve() }
        })
    }

    const loadIncrementalHistory = async (nid: string, startTime: number, endTime: number) => {
        const db = await initDBRead();
        return new Promise<void>((resolve, reject) => {
            isLoading.value = true
            const tx = db.transaction('history', 'readonly');
            const store = tx.objectStore('history');
            const index = store.index('nid_timestamp');

            const range = IDBKeyRange.bound([nid, startTime], [nid, endTime]);
            const request = index.getAll(range);

            request.onsuccess = () => {
                const results: RobotHistoryEntry[] = request.result;
                if (results && results.length > 0) {
                    dbLoadCount.value += results.length
                    const map = cache.value;
                    let history = map.get(nid);
                    if (!history) {
                        history = [];
                        map.set(nid, history);
                    }
                    results.forEach(entry => insertIntoCacheOptimized(history!, entry));
                    triggerRef(cache);
                }
                isLoading.value = false
                resolve();
            };
            request.onerror = (e) => {
                isLoading.value = false
                reject(e);
            }
        });
    }

    const ensureDroneDataLoaded = async (nid: string, centerTime: number, totalWindowMs = 60_000) => {
        const bufferMultiplier = 3
        const effectiveWindow = totalWindowMs * bufferMultiplier
        const start = centerTime - effectiveWindow / 2
        const end = centerTime + effectiveWindow / 2

        const history = cache.value.get(nid)
        if (history && history.length > 0) {
            const minCached = history[0].timestamp;
            const maxCached = history[history.length - 1].timestamp;
            if (minCached <= start && maxCached >= end) return;
        }
        await loadIncrementalHistory(nid, start, end)
    }

    const getSnapshotAt = async (nid: string, timestamp: number, windowSizeMs = 2500): Promise<RobotHistoryEntry | undefined> => {
        let history = cache.value.get(nid)
        if (history && history.length > 0) {
            const idx = findClosestIndex(history, timestamp)
            const entry = history[idx]
            if (Math.abs(entry.timestamp - timestamp) <= windowSizeMs) return entry
        }
        await ensureDroneDataLoaded(nid, timestamp, windowSizeMs * 4)
        history = cache.value.get(nid)
        if (history && history.length > 0) {
            const idx = findClosestIndex(history, timestamp)
            const entry = history[idx]
            if (Math.abs(entry.timestamp - timestamp) <= windowSizeMs) return entry
        }
        return undefined
    }

    const getDronesInWindow = async (centerTime: number, windowMs: number): Promise<Map<string, Robot>> => {
        const result = new Map<string, Robot>()
        const tolerance = windowMs / 2
        for (const nid of knownNids) {
            const entry = await getSnapshotAt(nid, centerTime, tolerance)
            if (entry) result.set(nid, entry.data)
        }
        return result
    }

    return {
        cache,
        activeRobotCount,
        isLoading,
        dbLoadCount,
        knownNids,
        addRobotsBatch,
        getSnapshotAt,
        getDronesInWindow,
        ensureDroneDataLoaded,
        discoverAvailableDrones
    }
})