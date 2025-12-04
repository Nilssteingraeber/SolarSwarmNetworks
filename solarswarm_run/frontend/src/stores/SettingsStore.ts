import { Cesium3DTileset } from 'cesium'
import { defineStore } from 'pinia'
import { ref } from 'vue'

export const useSettingsStore = defineStore('settings', () => {

    const show3dMesh = ref(true)
    const tileset = ref()
    const osmMap = ref()

    function setTileset(value: Cesium3DTileset) {
        tileset.value = value;
    }
    function getTileset() {
        return tileset.value;
    }

    function setOsmMap(value: Cesium3DTileset) {
        osmMap.value = value;
    }
    function getOsmMap() {
        return osmMap.value;
    }

    function setShow3dMesh(value: boolean) {
        show3dMesh.value = value
        tileset.value.show = show3dMesh.value;
    }

    function getShow3dMesh(): boolean {
        return show3dMesh.value
    }

    return {
        show3dMesh,
        setShow3dMesh,
        getShow3dMesh,
        setTileset,
        getTileset,
        setOsmMap,
        getOsmMap,
    }
})
