// src/workers/db.worker.ts

let _db: IDBDatabase | null = null;

async function initDB(): Promise<IDBDatabase> {
    // If we have a healthy connection, return it
    if (_db) return _db;

    return new Promise((resolve, reject) => {
        // Increment version to trigger onupgradeneeded for new stores
        const req = indexedDB.open('DroneHistoryDB', 5);

        req.onupgradeneeded = (e: any) => {
            const db = e.target.result;

            // 1. History Store
            if (!db.objectStoreNames.contains('history')) {
                const store = db.createObjectStore('history', { keyPath: ['nid', 'timestamp'] });
                store.createIndex('nid_timestamp', ['nid', 'timestamp']);
            }

            // 2. Geo Shapes Store
            if (!db.objectStoreNames.contains('geo_shapes')) {
                db.createObjectStore('geo_shapes', { keyPath: 'id' });
            }
        };

        req.onsuccess = () => {
            _db = req.result;

            _db.onclose = () => { _db = null; };
            _db.onversionchange = () => {
                _db?.close();
                _db = null;
            };

            resolve(_db);
        };
        req.onerror = (e) => reject(e);
    });
}

const CHUNK_SIZE = 1000;

self.onmessage = async (e: MessageEvent) => {
    const { type, payload } = e.data;

    try {
        const db = await initDB();

        switch (type) {
            case 'WRITE_BATCH':
                try {
                    for (let i = 0; i < payload.length; i += CHUNK_SIZE) {
                        const chunk = payload.slice(i, i + CHUNK_SIZE);

                        await new Promise<void>((resolve, reject) => {
                            const tx = db.transaction('history', 'readwrite');
                            const store = tx.objectStore('history');

                            chunk.forEach((entry: any) => store.put(entry));

                            tx.oncomplete = () => resolve();
                            tx.onerror = (err) => reject(err);
                            tx.onabort = () => reject(new Error("Transaction Aborted"));
                        });

                        const progress = Math.round(((i + chunk.length) / payload.length) * 100);
                        postMessage({ type: 'WRITE_PROGRESS', payload: progress });
                    }
                    postMessage({ type: 'WRITE_COMPLETE' });
                } catch (error: any) {
                    _db = null;
                    const msg = error?.message || String(error);
                    postMessage({ type: 'WRITE_ERROR', payload: msg });
                }
                break;

            case 'SAVE_SHAPE': {
                const tx = db.transaction('geo_shapes', 'readwrite');
                tx.objectStore('geo_shapes').put(payload);
                break;
            }

            case 'DELETE_SHAPE': {
                const tx = db.transaction('geo_shapes', 'readwrite');
                tx.objectStore('geo_shapes').delete(payload);
                break;
            }

            case 'CLEAR_SHAPES': {
                const tx = db.transaction('geo_shapes', 'readwrite');
                tx.objectStore('geo_shapes').clear();
                break;
            }

            case 'GET_ALL_SHAPES': {
                const tx = db.transaction('geo_shapes', 'readonly');
                const req = tx.objectStore('geo_shapes').getAll();
                req.onsuccess = () => {
                    postMessage({ type: 'ALL_SHAPES_RESULT', payload: req.result });
                };
                req.onerror = (err) => {
                    throw err;
                };
                break;
            }
        }
    } catch (error: any) {
        _db = null;
        console.error("Worker Global Error:", error);

        // Ensure the error sent to main thread is a plain string/object
        const errorMessage = error?.message || String(error);
        postMessage({ type: 'WORKER_ERROR', payload: errorMessage });
    }
};

export { };