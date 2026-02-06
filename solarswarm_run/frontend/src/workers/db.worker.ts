// src/workers/db.worker.ts

let _db: IDBDatabase | null = null
const DB_NAME = 'DroneHistoryDB'
const DB_VERSION = 5

const STORES = {
    HISTORY: 'history',
    GEO: 'geo_shapes',
} as const

function validateSchema(db: IDBDatabase) {
    if (
        !db.objectStoreNames.contains(STORES.HISTORY) ||
        !db.objectStoreNames.contains(STORES.GEO)
    ) {
        db.close()
        indexedDB.deleteDatabase(DB_NAME)
        throw new Error('IndexedDB schema mismatch')
    }
}

async function initDB(): Promise<IDBDatabase> {
    if (_db) {
        validateSchema(_db)
        return _db
    }

    return new Promise((resolve, reject) => {
        const req = indexedDB.open(DB_NAME, DB_VERSION)

        req.onupgradeneeded = (e: IDBVersionChangeEvent) => {
            const db = (e.target as IDBOpenDBRequest).result

            if (!db.objectStoreNames.contains(STORES.HISTORY)) {
                const store = db.createObjectStore(STORES.HISTORY, {
                    keyPath: ['nid', 'timestamp'],
                })
                store.createIndex('nid_timestamp', ['nid', 'timestamp'])
            }

            if (!db.objectStoreNames.contains(STORES.GEO)) {
                db.createObjectStore(STORES.GEO, { keyPath: 'id' })
            }
        }

        req.onsuccess = () => {
            const db = req.result
            try {
                validateSchema(db)
            } catch (err) {
                reject(err)
                return
            }

            _db = db
            _db.onclose = () => { _db = null }
            _db.onversionchange = () => {
                _db?.close()
                _db = null
            }

            resolve(_db)
        }

        req.onerror = () => reject(req.error)
    })
}

const CHUNK_SIZE = 1000

self.onmessage = async (e: MessageEvent) => {
    const { type, payload } = e.data

    try {
        const db = await initDB()

        switch (type) {
            case 'WRITE_BATCH':
                for (let i = 0; i < payload.length; i += CHUNK_SIZE) {
                    const chunk = payload.slice(i, i + CHUNK_SIZE)

                    await new Promise<void>((resolve, reject) => {
                        const tx = db.transaction(STORES.HISTORY, 'readwrite')
                        const store = tx.objectStore(STORES.HISTORY)

                        chunk.forEach((entry: any) => store.put(entry))

                        tx.oncomplete = () => resolve()
                        tx.onerror = () => reject(tx.error)
                        tx.onabort = () => reject(tx.error)
                    })

                    const progress = Math.round(((i + chunk.length) / payload.length) * 100)
                    postMessage({ type: 'WRITE_PROGRESS', payload: progress })
                }

                postMessage({ type: 'WRITE_COMPLETE' })
                break

            case 'SAVE_SHAPE': {
                const tx = db.transaction(STORES.GEO, 'readwrite')
                tx.objectStore(STORES.GEO).put(payload)
                break
            }

            case 'DELETE_SHAPE': {
                const tx = db.transaction(STORES.GEO, 'readwrite')
                tx.objectStore(STORES.GEO).delete(payload)
                break
            }

            case 'CLEAR_SHAPES': {
                const tx = db.transaction(STORES.GEO, 'readwrite')
                tx.objectStore(STORES.GEO).clear()
                break
            }

            case 'GET_ALL_SHAPES': {
                const tx = db.transaction(STORES.GEO, 'readonly')
                const req = tx.objectStore(STORES.GEO).getAll()
                req.onsuccess = () => {
                    postMessage({ type: 'ALL_SHAPES_RESULT', payload: req.result })
                }
                req.onerror = () => {
                    postMessage({ type: 'WORKER_ERROR', payload: req.error?.message })
                }
                break
            }
        }
    } catch (error: any) {
        _db = null
        postMessage({
            type: 'WORKER_ERROR',
            payload: error?.message || String(error),
        })
    }
}

export { }
