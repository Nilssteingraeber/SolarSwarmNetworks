import { ref, type Ref, shallowRef, shallowReactive, triggerRef } from 'vue'
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
    const db: Ref<IDBDatabase | null> = ref(null)
    const dbPromise: Ref<Promise<IDBDatabase> | null> = ref(null)
    const maxEntriesPerDrone = ref(5000)
    const cache = shallowRef(new Map<string, RobotHistoryEntry[]>())
    const pendingLoads = new Map<string, Promise<void>>()

    const cacheCenterTime = ref(useTimeStore().currentTime)
    const cacheWindowMs = ref(60_000)
    const timeStore = useTimeStore()

    const initDB = async (): Promise<IDBDatabase> => {
        if (db.value) return db.value
        if (dbPromise.value) return dbPromise.value
        dbPromise.value = new Promise((resolve, reject) => {
            const request = indexedDB.open('DroneHistoryDB', 1)
            request.onupgradeneeded = (e) => {
                const dbRes = (e.target as IDBOpenDBRequest).result
                if (!dbRes.objectStoreNames.contains('robots')) {
                    const store = dbRes.createObjectStore('robots', { keyPath: ['nid', 'timestamp'] })
                    store.createIndex('nid_timestamp', ['nid', 'timestamp'])
                    store.createIndex('nid', 'nid')
                }
            }
            request.onsuccess = () => { db.value = request.result; resolve(db.value) }
            request.onerror = () => { dbPromise.value = null; reject(request.error) }
        })
        return dbPromise.value
    }

    const insertIntoCache = (nid: string, entry: RobotHistoryEntry) => {
        let history = cache.value.get(nid)
        if (!history) {
            history = shallowReactive<RobotHistoryEntry[]>([])
            cache.value.set(nid, history)
            triggerRef(cache)
        }
        if (history.length === 0 || entry.timestamp > history[history.length - 1].timestamp) {
            history.push(entry)
        } else {
            const idx = findInsertionIndex(history, entry.timestamp)
            if (idx !== -1) history.splice(idx, 0, entry)
        }
    }

    const pruneCacheToWindow = (targetNid?: string) => {
        const start = cacheCenterTime.value - cacheWindowMs.value / 2
        const max = maxEntriesPerDrone.value
        const prune = (h: RobotHistoryEntry[]) => {
            let count = 0
            while (count < h.length && h[count].timestamp < start) count++
            if (count > 0) h.splice(0, count)
            if (h.length > max) h.splice(0, h.length - max)
        }
        if (targetNid) {
            const h = cache.value.get(targetNid); if (h) prune(h)
        } else {
            cache.value.forEach(prune)
        }
    }

    const loadIncrementalHistory = async (nid: string, startTime: number, endTime: number) => {
        const loadKey = `${nid}-${startTime}-${endTime}`
        if (pendingLoads.has(loadKey)) return pendingLoads.get(loadKey)

        const loadPromise = (async () => {
            const dbConn = await initDB()
            const tx = dbConn.transaction('robots', 'readonly')
            const store = tx.objectStore('robots')
            const index = store.index('nid_timestamp')
            const range = IDBKeyRange.bound([nid, startTime], [nid, endTime])

            return new Promise<void>((resolve, reject) => {
                const request = index.openCursor(range)
                request.onsuccess = (e) => {
                    const cursor = (e.target as IDBRequest).result as IDBCursorWithValue | null
                    if (!cursor) return resolve()
                    insertIntoCache(cursor.value.nid, cursor.value)
                    cursor.continue()
                }
                request.onerror = () => reject(request.error)
            })
        })()
        pendingLoads.set(loadKey, loadPromise)
        try { await loadPromise } finally { pendingLoads.delete(loadKey) }
    }

    const getSnapshotAt = async (nid: string, timestamp: number, windowSizeMs?: number): Promise<RobotHistoryEntry | undefined> => {
        const toleranceMs = windowSizeMs ?? 2500
        const history = cache.value.get(nid)
        if (history && history.length > 0) {
            const idx = findClosestIndex(history, timestamp)
            const entry = history[idx]
            if (Math.abs(entry.timestamp - timestamp) <= toleranceMs) return entry
        }
        await loadIncrementalHistory(nid, timestamp - toleranceMs, timestamp + toleranceMs)
        const updated = cache.value.get(nid)
        if (updated && updated.length > 0) {
            const idx = findClosestIndex(updated, timestamp)
            const entry = updated[idx]
            if (Math.abs(entry.timestamp - timestamp) <= toleranceMs) return entry
        }
        return undefined
    }

    /**
     * Iterates through all known drones and returns their state at the given time.
     */
    const getDronesInWindow = async (centerTime: number, windowMs: number): Promise<Map<string, Robot>> => {
        const result = new Map<string, Robot>()
        const tolerance = windowMs / 2

        // We use the existing cache keys to see which drones we know about
        for (const nid of cache.value.keys()) {
            const entry = await getSnapshotAt(nid, centerTime, tolerance)
            if (entry) {
                result.set(nid, entry.data)
            }
        }
        return result
    }

    const addRobotsBatch = async (robots: Robot[], timestamp?: number) => {
        if (!robots.length) return
        const ts = timestamp ?? Date.now()
        robots.forEach(r => {
            insertIntoCache(r.nid, { nid: r.nid, timestamp: ts, data: r })
            pruneCacheToWindow(r.nid)
        })
        try {
            const dbConn = await initDB()
            const tx = dbConn.transaction('robots', 'readwrite')
            const store = tx.objectStore('robots')
            robots.forEach(r => store.put({ nid: r.nid, timestamp: ts, data: r }))
            return new Promise((res, rej) => { tx.oncomplete = () => res(true); tx.onerror = () => rej(tx.error) })
        } catch (e) { console.warn("DB Write Error", e) }
    }

    const ensureDroneDataLoaded = async (nid: string, centerTime: number, totalWindowMs = 60_000) => {
        const start = centerTime - totalWindowMs / 2, end = centerTime + totalWindowMs / 2
        const h = cache.value.get(nid)
        if (h && h.length > 0 && h[0].timestamp <= start && h[h.length - 1].timestamp >= end) return
        await loadIncrementalHistory(nid, start, end)
    }

    return {
        cache,
        cacheCenterTime,
        cacheWindowMs,
        getSnapshotAt,
        getDronesInWindow, // Added
        addRobotsBatch,
        ensureDroneDataLoaded,
        loadIncrementalHistory // Added for flight path usage
    }
})