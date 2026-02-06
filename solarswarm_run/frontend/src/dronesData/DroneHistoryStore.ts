import { ref, type Ref, shallowRef, triggerRef, reactive } from 'vue'
import { defineStore } from 'pinia'
import type { Robot } from '../models/Robot'

export interface RobotHistoryEntry {
    timestamp: number
    nid: string
    data: Robot
}

export interface TimeRange {
    start: number
    end: number
}

// Enhanced Debug Statistics
export interface DebugStats {
    totalPointsInRam: number
    dbReads: number
    activeDrones: number
    memoryUsageMb: string // New: Estimated RAM usage
    cacheWindowStart: number | null
    cacheWindowEnd: number | null
}

let _db: IDBDatabase | null = null;

export const initDBRead = async (): Promise<IDBDatabase> => {
    if (_db) return _db;
    return new Promise((resolve, reject) => {
        const req = indexedDB.open('DroneHistoryDB', 5);
        req.onupgradeneeded = (e: any) => {
            const db = e.target.result;
            if (!db.objectStoreNames.contains('history')) {
                const store = db.createObjectStore('history', { keyPath: ['nid', 'timestamp'] });
                store.createIndex('nid_timestamp', ['nid', 'timestamp']);
            }
        };
        req.onsuccess = () => {
            _db = req.result;
            _db.onclose = () => { _db = null; };
            _db.onversionchange = () => { _db?.close(); _db = null; };
            resolve(_db);
        };
        req.onerror = (e) => { _db = null; reject(e); };
    });
};

// --- BINARY SEARCH UTILITIES ---
export function findLastIndexBefore(history: RobotHistoryEntry[], target: number): number {
    if (history.length === 0) return -1;
    if (target < history[0].timestamp) return -1;
    let left = 0, right = history.length - 1, result = -1;
    while (left <= right) {
        const mid = (left + right) >>> 1;
        if (history[mid].timestamp <= target) { result = mid; left = mid + 1; }
        else { right = mid - 1; }
    }
    return result;
}

export function findClosestIndex(history: RobotHistoryEntry[], target: number): number {
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

export function findInsertionIndex(history: RobotHistoryEntry[], timestamp: number): number {
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
    const worker = new Worker(
        new URL('../workers/db.worker.ts', import.meta.url),
        { type: 'module' }
    );

    // --- STATE ---
    const activityMarkers = ref<Map<string, number[]>>(new Map());

    // Only track what is IN MEMORY (The Green Zone)
    const cachedRanges = ref<TimeRange[]>([]);

    const debugStats = reactive<DebugStats>({
        totalPointsInRam: 0,
        dbReads: 0,
        activeDrones: 0,
        memoryUsageMb: "0.00",
        cacheWindowStart: null,
        cacheWindowEnd: null
    });

    // --- CONFIGURATION ---
    const config = reactive({
        maxCacheWindowMs: 30 * 1000,      // Keep +/- 30s around playhead
        loadingLookaheadMs: 5 * 1000,     // Trigger load when 5s from edge
        cacheContinuityLimitMs: 2000      // Visual merging tolerance
    });

    const cache = shallowRef(new Map<string, RobotHistoryEntry[]>())
    const isLoading = ref(false)
    const activeRobotCount = ref(0)
    const knownNids = reactive(new Set<string>())

    // --- INTERNAL HELPERS ---

    const updateCacheStats = () => {
        const intervals: TimeRange[] = [];
        let totalPoints = 0;

        for (const history of cache.value.values()) {
            const count = history.length;
            totalPoints += count;
            if (count > 0) {
                intervals.push({
                    start: history[0].timestamp,
                    end: history[count - 1].timestamp
                });
            }
        }

        // Approx 160 bytes per robot object + map overhead
        const bytes = totalPoints * 160;
        debugStats.memoryUsageMb = (bytes / 1024 / 1024).toFixed(2);
        debugStats.totalPointsInRam = totalPoints;
        debugStats.activeDrones = cache.value.size;

        if (intervals.length === 0) {
            cachedRanges.value = [];
            debugStats.cacheWindowStart = null;
            debugStats.cacheWindowEnd = null;
            return;
        }

        // Merge Logic
        intervals.sort((a, b) => a.start - b.start);
        const merged: TimeRange[] = [];
        let current = intervals[0];
        let minGlobal = current.start;
        let maxGlobal = current.end;

        for (let i = 1; i < intervals.length; i++) {
            const next = intervals[i];
            minGlobal = Math.min(minGlobal, next.start);
            maxGlobal = Math.max(maxGlobal, next.end);

            if (next.start <= current.end + config.cacheContinuityLimitMs) {
                current.end = Math.max(current.end, next.end);
            } else {
                merged.push(current);
                current = next;
            }
        }
        merged.push(current);

        cachedRanges.value = merged;
        debugStats.cacheWindowStart = minGlobal;
        debugStats.cacheWindowEnd = maxGlobal;
    }

    const insertIntoCacheOptimized = (history: RobotHistoryEntry[], entry: RobotHistoryEntry) => {
        const len = history.length;
        if (len === 0 || entry.timestamp >= history[len - 1].timestamp) {
            history.push(entry);
            return;
        }
        const idx = findInsertionIndex(history, entry.timestamp);
        if (idx !== -1) history.splice(idx, 0, entry);
    }

    // --- STRICT PRUNING (Debounced) ---
    let pruneDebounce: number | null = null;

    const enforceCacheWindow = (centerTime: number) => {
        if (pruneDebounce) return;

        const minAllowed = centerTime - config.maxCacheWindowMs;
        const maxAllowed = centerTime + config.maxCacheWindowMs;

        let didChange = false;

        for (const history of cache.value.values()) {
            if (history.length === 0) continue;

            let removeStart = 0;
            while (removeStart < history.length && history[removeStart].timestamp < minAllowed) {
                removeStart++;
            }
            if (removeStart > 0) {
                history.splice(0, removeStart);
                didChange = true;
            }

            let removeEnd = 0;
            while (history.length > 0 && history[history.length - 1].timestamp > maxAllowed) {
                history.pop();
                removeEnd++;
                didChange = true;
            }
        }
        if (didChange) updateCacheStats();

        pruneDebounce = window.setTimeout(() => pruneDebounce = null, 200);
    }

    // --- MAIN API ---

    const addRobotsBatch = (robots: Robot[], timestamp?: number) => {
        if (!robots.length) return
        const ts = timestamp ?? Date.now()
        activeRobotCount.value = robots.length



        const dbEntries = new Array(robots.length)
        const map = cache.value
        let ramCacheUpdated = false;


        for (let i = 0; i < robots.length; i++) {
            const r = robots[i]
            const entry = { nid: r.nid, timestamp: ts, data: r }
            dbEntries[i] = entry
            if (!knownNids.has(r.nid)) knownNids.add(r.nid)

            let history = map.get(r.nid)

            if (!history) {
                history = []
                map.set(r.nid, history)
                history.push(entry)
                ramCacheUpdated = true;
            } else {
                if (history.length > 0) {
                    const lastTs = history[history.length - 1].timestamp;
                    const firstTs = history[0].timestamp;
                    const isConnected = (ts >= firstTs - 5000) && (ts <= lastTs + 5000);
                    if (isConnected) {
                        insertIntoCacheOptimized(history, entry);
                        ramCacheUpdated = true;
                    }
                } else {
                    history.push(entry);
                    ramCacheUpdated = true;
                }
            }
        }

        if (ramCacheUpdated) {
            triggerRef(cache)
            updateCacheStats()
        }
        worker.postMessage({ type: 'WRITE_BATCH', payload: dbEntries })
    }

    const loadIncrementalHistory = async (nid: string, startTime: number, endTime: number) => {
        const db = await initDBRead();
        return new Promise<void>((resolve, reject) => {
            isLoading.value = true;
            try {
                const tx = db.transaction('history', 'readonly');
                const store = tx.objectStore('history');
                const index = store.index('nid_timestamp');
                const range = IDBKeyRange.bound([nid, startTime], [nid, endTime]);
                const request = index.getAll(range);

                request.onsuccess = () => {
                    const results: RobotHistoryEntry[] = request.result;
                    if (results && results.length > 0) {
                        debugStats.dbReads++;
                        const map = cache.value;
                        let history = map.get(nid);
                        if (!history) {
                            history = [];
                            map.set(nid, history);
                        }
                        results.forEach(entry => insertIntoCacheOptimized(history!, entry));
                        triggerRef(cache);
                        updateCacheStats();
                    }
                    isLoading.value = false;
                    resolve();
                };
                request.onerror = (e) => { isLoading.value = false; reject(e); };
            } catch (err) { isLoading.value = false; reject(err); }
        });
    };

    const ensureDroneDataLoaded = async (nid: string, centerTime: number) => {
        enforceCacheWindow(centerTime);

        const start = centerTime - config.maxCacheWindowMs;
        const end = centerTime + config.maxCacheWindowMs;

        const history = cache.value.get(nid);
        let needLoad = true;

        if (history && history.length > 0) {
            const cacheStart = history[0].timestamp;
            const cacheEnd = history[history.length - 1].timestamp;

            const isStartCovered = cacheStart <= (start + 2000);
            const isEndCovered = cacheEnd >= (end - config.loadingLookaheadMs);

            if (isStartCovered && isEndCovered) {
                needLoad = false;
            }
        }

        if (needLoad) {
            await loadIncrementalHistory(nid, start, end);
        }
    }

    const getSnapshotAt = async (nid: string, timestamp: number, windowSizeMs = 2500): Promise<RobotHistoryEntry | undefined> => {
        let history = cache.value.get(nid)
        if (history && history.length > 0) {
            const idx = findClosestIndex(history, timestamp)
            const entry = history[idx]
            if (Math.abs(entry.timestamp - timestamp) <= windowSizeMs) return entry
        }
        await ensureDroneDataLoaded(nid, timestamp)
        history = cache.value.get(nid)
        if (history && history.length > 0) {
            const idx = findClosestIndex(history, timestamp)
            const entry = history[idx]
            if (Math.abs(entry.timestamp - timestamp) <= windowSizeMs) return entry
        }
        return undefined
    }

    const loadActivityMarkers = async (startTime: number, endTime: number) => {
        const actualStart = Math.min(startTime, endTime);
        const actualEnd = Math.max(startTime, endTime);
        if (actualEnd - actualStart < 1000) return;
        isLoading.value = true;
        const db = await initDBRead();
        const newMarkers = new Map<string, number[]>();
        const targetPoints = 600;
        const rawRes = (actualEnd - actualStart) / targetPoints;
        const resolutionMs = Math.max(1000, Math.floor(rawRes));
        return new Promise<void>((resolve) => {
            try {
                const tx = db.transaction('history', 'readonly');
                const store = tx.objectStore('history');
                const index = store.index('nid_timestamp');
                const range = IDBKeyRange.lowerBound(['', actualStart]);
                const request = index.openCursor(range);
                request.onsuccess = (e: any) => {
                    const cursor = e.target.result;
                    if (!cursor) {
                        activityMarkers.value = newMarkers;
                        isLoading.value = false;
                        resolve();
                        return;
                    }
                    const [nid, ts] = cursor.key;
                    if (ts < actualStart) { cursor.continue([nid, actualStart]); return; }
                    if (ts > actualEnd) { cursor.continue([nid + '\u0000', actualStart]); return; }
                    if (!newMarkers.has(nid)) newMarkers.set(nid, []);
                    newMarkers.get(nid)!.push(ts);
                    cursor.continue([nid, ts + resolutionMs]);
                };
                request.onerror = () => { isLoading.value = false; resolve(); };
            } catch (err) { isLoading.value = false; resolve(); }
        });
    };

    // Stubs
    const discoverAvailableDrones = async () => {
        const db = await initDBRead();
        return new Promise<void>((resolve) => {
            const tx = db.transaction('history', 'readonly');
            const store = tx.objectStore('history');
            const req = store.openCursor();
            const seen = new Set<string>();
            req.onsuccess = (e: any) => {
                const cursor = e.target.result;
                if (cursor) { if (!seen.has(cursor.value.nid)) { seen.add(cursor.value.nid); knownNids.add(cursor.value.nid); } cursor.continue(); } else resolve();
            }
        });
    }
    const getNextPointAfter = async (nid: string, t: number, w: number) => { return null; }
    const getDronesInWindow = async (center: number, win: number) => {
        const result = new Map<string, Robot>()
        for (const nid of knownNids) {
            const entry = await getSnapshotAt(nid, center, win / 2)
            if (entry) result.set(nid, entry.data)
        }
        return result
    }
    const exportDatabaseToFile = async () => { /* impl */ }
    const importDatabaseFromFile = async (file: File) => { /* impl */ }

    return {
        cache,
        isLoading,
        cachedRanges,
        config,
        debugStats,
        knownNids,
        activeRobotCount,
        addRobotsBatch,
        getSnapshotAt,
        getNextPointAfter,
        getDronesInWindow,
        ensureDroneDataLoaded,
        discoverAvailableDrones,
        activityMarkers,
        loadActivityMarkers,
        exportDatabaseToFile,
        importDatabaseFromFile,
        initDBRead,
        findClosestIndex,
        findLastIndexBefore,
        findInsertionIndex
    }
})