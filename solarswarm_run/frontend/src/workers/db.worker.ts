// src/workers/db.worker.ts

let db: IDBDatabase | null = null;

const initDB = (): Promise<IDBDatabase> => {
    if (db) return Promise.resolve(db);
    return new Promise((resolve, reject) => {
        // CRITICAL: Must be Version 2 to match Store
        const request = indexedDB.open('DroneHistoryDB', 2);

        request.onupgradeneeded = (e) => {
            const dbRes = (e.target as IDBOpenDBRequest).result;
            // CRITICAL: Must use 'history' to match Store
            if (!dbRes.objectStoreNames.contains('history')) {
                const store = dbRes.createObjectStore('history', { keyPath: ['nid', 'timestamp'] });
                store.createIndex('nid_timestamp', ['nid', 'timestamp']);
            }
        };
        request.onsuccess = () => {
            db = request.result;
            resolve(db);
        };
        request.onerror = () => reject(request.error);
    });
};

self.onmessage = async (e: MessageEvent) => {
    const { type, payload } = e.data;

    if (type === 'WRITE_BATCH') {
        try {
            const database = await initDB();
            // CRITICAL: Must write to 'history'
            const tx = database.transaction('history', 'readwrite');
            const store = tx.objectStore('history');

            payload.forEach((entry: any) => {
                store.put(entry);
            });

            tx.oncomplete = () => postMessage({ type: 'WRITE_COMPLETE' });
            tx.onerror = (err) => console.error("Worker DB Transaction Error", err);
        } catch (error) {
            console.error("Worker Error", error);
        }
    }
};

export { };