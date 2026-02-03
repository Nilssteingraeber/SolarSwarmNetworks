import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import type { ParsedInterface } from '../DronesData/NodeRosInterfaces'

export type NamedParsedInterface = {
    serviceName: string
    serviceType: string
    parsed: ParsedInterface
}

type CacheEntry = {
    value: NamedParsedInterface[]
    expiresAt: number
}

const TTL_MS = 5 * 60 * 1000

export const useServiceInterfaceStore = defineStore(
    'serviceInterfaces',
    () => {
        // --------------------
        // state
        // --------------------

        // key -> cached service list
        const cache = ref<Map<string, CacheEntry>>(new Map())

        // --------------------
        // helpers
        // --------------------

        function isExpired(entry: CacheEntry) {
            return Date.now() > entry.expiresAt
        }

        function cleanupExpired(key?: string) {
            if (key) {
                const entry = cache.value.get(key)
                if (entry && isExpired(entry)) {
                    cache.value.delete(key)
                }
                return
            }

            // full cleanup
            for (const [k, entry] of cache.value.entries()) {
                if (isExpired(entry)) {
                    cache.value.delete(k)
                }
            }
        }

        // --------------------
        // getters
        // --------------------

        const keys = computed(() => Array.from(cache.value.keys()))

        const has = (key: string) =>
            computed(() => {
                cleanupExpired(key)
                return cache.value.has(key)
            })

        const get = (key: string) =>
            computed<NamedParsedInterface[] | undefined>(() => {
                cleanupExpired(key)
                return cache.value.get(key)?.value
            })

        const getByServiceName = (key: string, serviceName: string) =>
            computed(() => {
                cleanupExpired(key)
                return cache.value
                    .get(key)
                    ?.value.find(s => s.serviceName === serviceName)
            })

        const getByServiceType = (key: string, serviceType: string) =>
            computed(() => {
                cleanupExpired(key)
                return cache.value
                    .get(key)
                    ?.value.filter(s => s.serviceType === serviceType) ?? []
            })

        // --------------------
        // actions
        // --------------------

        function set(key: string, services: NamedParsedInterface[]) {
            cache.value.set(key, {
                value: services,
                expiresAt: Date.now() + TTL_MS,
            })
        }

        function addOrUpdate(
            key: string,
            service: NamedParsedInterface
        ) {
            const entry = cache.value.get(key)

            if (!entry || isExpired(entry)) {
                cache.value.set(key, {
                    value: [service],
                    expiresAt: Date.now() + TTL_MS,
                })
                return
            }

            const idx = entry.value.findIndex(
                s => s.serviceName === service.serviceName
            )

            if (idx === -1) {
                entry.value.push(service)
            } else {
                entry.value[idx] = service
            }

            // refresh TTL
            entry.expiresAt = Date.now() + TTL_MS
        }

        function invalidate(key: string) {
            cache.value.delete(key)
        }

        function clear() {
            cache.value.clear()
        }

        // --------------------
        // exposed API
        // --------------------
        return {
            // state
            cache,

            // getters
            keys,
            has,
            get,
            getByServiceName,
            getByServiceType,

            // actions
            set,
            addOrUpdate,
            invalidate,
            clear,
        }
    }
)
