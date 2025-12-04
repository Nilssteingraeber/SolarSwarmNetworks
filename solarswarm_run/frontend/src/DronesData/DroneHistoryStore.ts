import { ref, Ref, computed } from 'vue'
import { defineStore } from 'pinia'
import type { Robot } from '../models/Robot'
import { useTimeStore } from '../stores/TimeStore'

export interface RobotHistoryEntry {
    timestamp: number
    nid: string
    data: Robot
}

export const useDroneHistoryStore = defineStore('droneHistory', () => {
    // Reactive state for DB connection and cache
    const db: Ref<IDBDatabase | null> = ref(null)
    const dbPromise: Ref<Promise<IDBDatabase> | null> = ref(null)

    const maxEntriesPerDrone: Ref<number> = ref(5000)

    // Cache structure: Map of drone NID → Map of timestamp → History Entry
    const cache: Ref<Map<string, Map<number, RobotHistoryEntry>>> = ref(new Map())

    // Cache time window tracking
    const cacheCenterTime: Ref<number> = ref(useTimeStore().currentTime)
    const cacheWindowMs: Ref<number> = ref(60_000)

    // Cache refresh control
    const lastCacheRefresh: Ref<number> = ref(0)
    const minRefreshInterval: Ref<number> = ref(100)

    // Flags
    const isPopulating: Ref<boolean> = ref(false)

    // Dependent global time store
    const timeStore = useTimeStore()

    // Computed properties
    const activeRobotCount = computed(() => cache.value.size)

    // -------------------------
    // DATABASE INIT
    // -------------------------
    const initDB = async (): Promise<IDBDatabase> => {
        if (db.value) return db.value
        if (dbPromise.value) return dbPromise.value

        dbPromise.value = new Promise<IDBDatabase>((resolve, reject) => {
            const request = indexedDB.open('DroneHistoryDB', 1)

            request.onupgradeneeded = (event) => {
                const target = event.target as IDBOpenDBRequest
                const db = target.result
                if (!db.objectStoreNames.contains('robots')) {
                    const store = db.createObjectStore('robots', { keyPath: ['nid', 'timestamp'] })
                    store.createIndex('nid', 'nid')
                    store.createIndex('timestamp', 'timestamp')
                    store.createIndex('nid_timestamp', ['nid', 'timestamp'])
                }
            }

            request.onsuccess = () => {
                db.value = request.result
                db.value.onversionchange = () => {
                    db.value?.close()
                    db.value = null
                    dbPromise.value = null
                }
                resolve(db.value!)
            }

            request.onerror = () => {
                dbPromise.value = null
                reject(request.error)
            }
        })

        return dbPromise.value
    }

    const findClosestSnapshot = (entries: Iterable<RobotHistoryEntry>, target: number): RobotHistoryEntry | undefined => {
        let best: RobotHistoryEntry | undefined
        let bestDelta = Infinity
        for (const entry of entries) {
            const delta = Math.abs(entry.timestamp - target)
            if (delta < bestDelta) {
                best = entry
                bestDelta = delta
            }
        }
        return best
    }




    const loadIncrementalHistory = async (nid: string, startTime: number, endTime: number) => {
        const dbConn = await initDB()
        const tx = dbConn.transaction('robots', 'readonly')
        const store = tx.objectStore('robots')
        const index = store.index('nid_timestamp') // instead of 'nid'
        const range = IDBKeyRange.bound([nid, startTime], [nid, endTime])

        const request = index.openCursor(range)
        const nidMap = cache.value.get(nid) ?? new Map()
        if (!cache.value.has(nid)) cache.value.set(nid, nidMap)

        return new Promise<void>((resolve, reject) => {
            request.onsuccess = (e) => {
                const cursor = (e.target as IDBRequest).result as IDBCursorWithValue | null
                if (!cursor) return resolve()
                const entry = cursor.value as RobotHistoryEntry
                nidMap.set(entry.timestamp, entry)
                cursor.continue()
            }
            request.onerror = () => reject(request.error)
        })
    }

    const getSnapshotAt = async (
        nid: string,
        timestamp: number,
        windowSizeMs?: number,
        existingRange: { start: number; end: number } | null = null
    ): Promise<Robot | undefined> => {
        const nidMap = cache.value.get(nid)
        const toleranceMs = windowSizeMs ?? cacheWindowMs.value
        let loadStart = timestamp - toleranceMs / 2
        let loadEnd = timestamp + toleranceMs / 2

        if (existingRange) {
            // Only load gaps outside existingRange
            if (timestamp < existingRange.start) loadEnd = existingRange.start
            else if (timestamp > existingRange.end) loadStart = existingRange.end
            else return nidMap ? findClosestSnapshot(nidMap.values(), timestamp)?.data : undefined
        }

        // Try cached snapshot first
        if (nidMap) {
            const cached = findClosestSnapshot(nidMap.values(), timestamp)
            if (cached) return cached.data
        }

        // Load missing history
        await loadIncrementalHistory(nid, loadStart, loadEnd)

        // Check again after loading
        const updatedMap = cache.value.get(nid)
        if (updatedMap) {
            const updated = findClosestSnapshot(updatedMap.values(), timestamp)
            if (updated) return updated.data
        }

        return undefined
    }


    const getAllDroneHistory = async (nid: string): Promise<RobotHistoryEntry[]> => {
        const dbConn = await initDB()
        const tx = dbConn.transaction('robots', 'readonly')
        const store = tx.objectStore('robots')
        const index = store.index('nid')

        const nidMap = cache.value.get(nid) ?? new Map()
        if (!cache.value.has(nid)) cache.value.set(nid, nidMap)

        // Determine cached timestamps
        const cachedTimes = Array.from(nidMap.keys()).sort((a, b) => a - b)
        const entries: RobotHistoryEntry[] = []

        // Helper to load a range [start, end] from IndexedDB
        const loadRange = (start: number, end: number) =>
            new Promise<void>((resolve, reject) => {
                const range = IDBKeyRange.bound([nid, start], [nid, end])
                const request = index.openCursor(range)

                request.onsuccess = (event) => {
                    const cursor = (event.target as IDBRequest).result as IDBCursorWithValue | null
                    if (!cursor) {
                        resolve()
                        return
                    }
                    const entry = cursor.value as RobotHistoryEntry
                    entries.push(entry)
                    nidMap.set(entry.timestamp, entry)
                    cursor.continue()
                }
                request.onerror = () => reject(request.error)
            })

        if (!cachedTimes.length) {
            // Nothing cached, load everything
            await loadRange(Number.MIN_SAFE_INTEGER, Number.MAX_SAFE_INTEGER)
        } else {
            // Compute missing ranges
            const ranges: { start: number; end: number }[] = []

            // Before first cached entry
            if (cachedTimes[0] > Number.MIN_SAFE_INTEGER) {
                ranges.push({ start: Number.MIN_SAFE_INTEGER, end: cachedTimes[0] })
            }

            // Between cached entries
            for (let i = 0; i < cachedTimes.length - 1; i++) {
                if (cachedTimes[i + 1] - cachedTimes[i] > 1) {
                    ranges.push({ start: cachedTimes[i], end: cachedTimes[i + 1] })
                }
            }

            // After last cached entry
            if (cachedTimes[cachedTimes.length - 1] < Number.MAX_SAFE_INTEGER) {
                ranges.push({ start: cachedTimes[cachedTimes.length - 1], end: Number.MAX_SAFE_INTEGER })
            }

            // Load only missing ranges
            for (const r of ranges) {
                await loadRange(r.start, r.end)
            }
        }

        return entries
    }




    // -------------------------
    // SINGLE ENTRY QUERY
    // -------------------------
    const getDroneEntryAtTime = async (nid: string, timestamp: number): Promise<RobotHistoryEntry | undefined> => {
        await expandCacheTowards(timestamp)

        const nidMap = cache.value.get(nid)
        if (!nidMap) return undefined

        let best: RobotHistoryEntry | undefined
        for (const entry of nidMap.values()) {
            if (entry.timestamp <= timestamp) {
                if (!best || entry.timestamp > best.timestamp) best = entry
            }
        }
        return best
    }

    const expandCacheTowards = async (timestamp: number): Promise<void> => {
        const halfWindow = cacheWindowMs.value / 2
        const preloadMargin = cacheWindowMs.value * 0.3
        const start = cacheCenterTime.value - halfWindow
        const end = cacheCenterTime.value + halfWindow

        if (timestamp < start + preloadMargin || timestamp > end - preloadMargin) {
            cacheCenterTime.value = timestamp
            await populateCache()
        }
    }

    const populateCache = async (): Promise<void> => {
        if (isPopulating.value) return
        if (ensureCacheValid()) return

        isPopulating.value = true
        lastCacheRefresh.value = Date.now()

        try {
            const dbConn = await initDB()
            const tx = dbConn.transaction('robots', 'readonly')
            const store = tx.objectStore('robots')
            const index = store.index('nid')

            const nidRequest = index.openKeyCursor()
            const allNids = new Set<string>()

            await new Promise<void>((resolve, reject) => {
                nidRequest.onsuccess = (event) => {
                    // @ts-expect-error
                    const cursor = (event.target?.result as IDBCursor) || null
                    if (!cursor) return resolve()
                    // @ts-expect-error
                    const nid = cursor?.primaryKey[0] as string
                    allNids.add(nid)
                    cursor.continue()
                }
                nidRequest.onerror = () => reject(nidRequest.error)
            })

            const windowStart = cacheCenterTime.value - cacheWindowMs.value / 2
            const windowEnd = cacheCenterTime.value + cacheWindowMs.value / 2

            for (const nid of allNids) {
                await loadIncrementalHistory(nid, windowStart, windowEnd)
            }
        } finally {
            isPopulating.value = false
        }
    }

    const ensureCacheValid = (): boolean => {
        const now = timeStore.currentTime
        const start = cacheCenterTime.value - cacheWindowMs.value / 2
        const end = cacheCenterTime.value + cacheWindowMs.value / 2
        const outsideWindow = now < start || now > end
        const cooldown = Date.now() - lastCacheRefresh.value < minRefreshInterval.value

        if (outsideWindow && !cooldown) {
            cacheCenterTime.value = now
            return false
        }
        return true
    }

    // -------------------------
    // CACHE MANAGEMENT
    // -------------------------
    const addRobot = async (robot: Robot, timestamp: number) => {
        const nidMap = cache.value.get(robot.nid) ?? new Map()
        if (!cache.value.has(robot.nid)) cache.value.set(robot.nid, nidMap)

        const entry: RobotHistoryEntry = { nid: robot.nid, timestamp, data: robot }
        nidMap.set(timestamp, entry)
        pruneCacheToWindow(robot.nid)

        if (nidMap.size > maxEntriesPerDrone.value) {
            const sortedKeys = Array.from(nidMap.keys()).sort((a, b) => a - b)
            const excess = sortedKeys.length - maxEntriesPerDrone.value
            for (let i = 0; i < excess; i++) nidMap.delete(sortedKeys[i])
        }

        try {
            const dbConn = await initDB()
            const tx = dbConn.transaction('robots', 'readwrite')
            const request = tx.objectStore('robots').put({ nid: robot.nid, timestamp, data: robot })
            await new Promise<void>((resolve, reject) => {
                request.onsuccess = () => resolve()
                request.onerror = () => reject(request.error)
            })
        } catch (e) { /* silent */ }
    }

    const addRobotsBatch = async (robots: Robot[], timestamp?: number) => {
        if (!robots.length) return
        const ts = timestamp ?? Date.now()

        for (const robot of robots) {
            const nidMap = cache.value.get(robot.nid) ?? new Map()
            if (!cache.value.has(robot.nid)) cache.value.set(robot.nid, nidMap)
            nidMap.set(ts, { nid: robot.nid, timestamp: ts, data: robot })
            pruneCacheToWindow(robot.nid)
        }

        try {
            const dbConn = await initDB()
            const tx = dbConn.transaction('robots', 'readwrite')
            const store = tx.objectStore('robots')
            await Promise.all(robots.map(robot => new Promise<void>((resolve, reject) => {
                const req = store.put({ nid: robot.nid, timestamp: ts, data: robot })
                req.onsuccess = () => resolve()
                req.onerror = () => reject(req.error)
            })))
        } catch (e) { /* silent */ }
    }

    const pruneCacheToWindow = (targetNid?: string) => {
        const start = cacheCenterTime.value - cacheWindowMs.value / 2
        const end = cacheCenterTime.value + cacheWindowMs.value / 2

        if (targetNid) {
            const nidMap = cache.value.get(targetNid)
            if (!nidMap) return
            for (const ts of nidMap.keys()) if (ts < start || ts > end) nidMap.delete(ts)
            if (nidMap.size === 0) cache.value.delete(targetNid)
        } else {
            for (const [nidKey, nidMap] of cache.value) {
                for (const ts of nidMap.keys()) if (ts < start || ts > end) nidMap.delete(ts)
                if (nidMap.size === 0) cache.value.delete(nidKey)
            }
        }
    }

    const clear = async () => {
        cache.value.clear()
        cacheCenterTime.value = 0

        const dbConn = await initDB()
        const tx = dbConn.transaction('robots', 'readwrite')
        const request = tx.objectStore('robots').clear()
        await new Promise<void>((resolve, reject) => {
            request.onsuccess = () => resolve()
            request.onerror = () => reject(request.error)
        })
    }

    const preloadAroundNow = async () => {
        cacheCenterTime.value = Date.now()
    }

    return {
        db,
        cache,
        cacheCenterTime,
        cacheWindowMs,
        activeRobotCount,

        initDB,
        getSnapshotAt,
        addRobot,
        addRobotsBatch,
        getDronesInWindow: async (centerTime: number, windowMs: number) => {
            const start = centerTime - windowMs / 2
            const end = centerTime + windowMs / 2
            cacheCenterTime.value = centerTime

            // Ensure cache is populated
            await populateCache()

            const result = new Map<string, Robot>()

            for (const [nid, nidMap] of cache.value) {
                // Check if there is already an entry in the desired window
                const hasDataInWindow = Array.from(nidMap.values()).some(
                    e => e.timestamp >= start && e.timestamp <= end
                )

                // If not, load missing range from IndexedDB
                if (!hasDataInWindow) {
                    await loadIncrementalHistory(nid, start, end)
                }

                // Now select the latest entry in the window
                const updatedMap = cache.value.get(nid)
                if (!updatedMap) continue

                let best: RobotHistoryEntry | undefined
                for (const entry of updatedMap.values()) {
                    if (entry.timestamp >= start && entry.timestamp <= end) {
                        if (!best || entry.timestamp > best.timestamp) best = entry
                    }
                }

                if (best) {
                    result.set(nid, best.data)
                }
            }

            return result
        },
        getDroneHistoryAroundTime: async (
            nid: string,
            targetTime: number,
            n: number
        ): Promise<RobotHistoryEntry[]> => {
            if (n <= 0) return [];

            const half = Math.floor(n / 2);

            // Ensure cache exists
            let nidMap = cache.value.get(nid);
            if (!nidMap) {
                nidMap = new Map<number, RobotHistoryEntry>();
                cache.value.set(nid, nidMap);
            }

            // Get already cached timestamps, sorted
            let cachedTimestamps = Array.from(nidMap.keys()).sort((a, b) => a - b);

            const firstCached = cachedTimestamps[0];
            const lastCached = cachedTimestamps[cachedTimestamps.length - 1];

            // Compute the range we need to cover around targetTime
            const desiredStart = targetTime - half * 1000;
            const desiredEnd = targetTime + half * 1000;

            // Load missing ranges from DB
            const loadPromises: Promise<void>[] = [];
            if (!firstCached || desiredStart < firstCached) {
                // Need earlier data
                const start = desiredStart;
                const end = firstCached ? firstCached - 1 : desiredEnd;
                loadPromises.push(loadIncrementalHistory(nid, start, end));
            }

            if (!lastCached || desiredEnd > lastCached) {
                // Need later data
                const start = lastCached ? lastCached + 1 : desiredStart;
                const end = desiredEnd;
                loadPromises.push(loadIncrementalHistory(nid, start, end));
            }

            if (loadPromises.length) await Promise.all(loadPromises);

            // Refresh sorted timestamps after DB load
            cachedTimestamps = Array.from(nidMap.keys()).sort((a, b) => a - b);

            // Binary search to find closest index <= targetTime
            let low = 0, high = cachedTimestamps.length - 1, centerIdx = -1;
            while (low <= high) {
                const mid = (low + high) >> 1;
                if (cachedTimestamps[mid] <= targetTime) {
                    centerIdx = mid;
                    low = mid + 1;
                } else {
                    high = mid - 1;
                }
            }

            // Collect closest n entries around targetTime
            const result: RobotHistoryEntry[] = [];
            let left = centerIdx;
            let right = centerIdx + 1;

            while (result.length < n && (left >= 0 || right < cachedTimestamps.length)) {
                const leftDiff = left >= 0 ? Math.abs(cachedTimestamps[left] - targetTime) : Infinity;
                const rightDiff = right < cachedTimestamps.length ? Math.abs(cachedTimestamps[right] - targetTime) : Infinity;

                if (leftDiff <= rightDiff) {
                    if (left >= 0) result.unshift(nidMap.get(cachedTimestamps[left])!);
                    left--;
                } else {
                    if (right < cachedTimestamps.length) result.push(nidMap.get(cachedTimestamps[right])!);
                    right++;
                }
            }

            return result;
        }

        ,
        getDroneEntryAtTime,
        clear,
        ensureDroneDataLoaded: async (nid: string, centerTime: number, totalWindowMs = 60_000) => {
            const halfWindow = totalWindowMs / 2
            const windowStart = centerTime - halfWindow
            const windowEnd = centerTime + halfWindow

            const nidMap = cache.value.get(nid)
            if (nidMap) {
                const cachedTimes = Array.from(nidMap.keys()).sort((a, b) => a - b)
                const cachedStart = cachedTimes[0] ?? 0
                const cachedEnd = cachedTimes[cachedTimes.length - 1] ?? 0

                if (cachedStart <= windowStart && cachedEnd >= windowEnd) return

                const gaps: { start: number; end: number }[] = []
                if (cachedStart > windowStart) gaps.push({ start: windowStart, end: Math.min(cachedStart, windowEnd) })
                if (cachedEnd < windowEnd) gaps.push({ start: Math.max(cachedEnd, windowStart), end: windowEnd })
                for (const gap of gaps) await loadIncrementalHistory(nid, gap.start, gap.end)
                return
            }

            await loadIncrementalHistory(nid, windowStart, windowEnd)
        },
        loadIncrementalHistory,
        getAllDroneHistory,
        preloadAroundNow,
        logDebugInfo: () => { },
    }
})
