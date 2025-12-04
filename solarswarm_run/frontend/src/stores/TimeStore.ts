import { defineStore } from 'pinia'

export const useTimeStore = defineStore('time', {
    state: () => ({
        currentTime: Date.now()
    }),

    actions: {
        setTime(t: number) {
            this.currentTime = t
        }
    }
})
