// src/services/GeoWorkerService.ts

import { GeoForm } from '../../models/GeoForm';

const worker = new Worker(
    new URL('../../workers/db.worker', import.meta.url),
    { type: 'module' }
);

export const geoWorkerService = {
    saveShape(form: GeoForm) {
        worker.postMessage({ type: 'SAVE_SHAPE', payload: form });
    },

    deleteShape(id: string) {
        worker.postMessage({ type: 'DELETE_SHAPE', payload: id });
    },

    clearAll() {
        worker.postMessage({ type: 'CLEAR_SHAPES' });
    },

    // Returns a Promise that resolves when the worker sends back the data
    getAllShapes(): Promise<GeoForm[]> {
        return new Promise((resolve) => {
            // 1. Create a one-time listener for the result
            const handler = (e: MessageEvent) => {
                if (e.data.type === 'ALL_SHAPES_RESULT') {
                    worker.removeEventListener('message', handler);
                    resolve(e.data.payload as GeoForm[]);
                }
            };

            worker.addEventListener('message', handler);

            // 2. Send the request
            worker.postMessage({ type: 'GET_ALL_SHAPES' });
        });
    }
};