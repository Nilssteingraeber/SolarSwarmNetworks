<script setup>
import { Vector3, TextureLoader, RepeatWrapping } from 'three'
import { ref, onMounted, watch } from 'vue'
import { TresCanvas } from '@tresjs/core'
import { CanvasTexture } from 'three'
import { OhVueIcon } from 'oh-vue-icons';
import { OrbitControls } from '@tresjs/cientos'
import {
    Group,
    CylinderGeometry,
    ConeGeometry,
    MeshStandardMaterial,
    Mesh,
    Quaternion
} from 'three'
import shadow from '@/assets/shadowmap.png'
import { MDBCol, MDBRow } from 'mdb-vue-ui-kit';


function createArrow(direction, origin, length, color, shaftRadius = 0.05, headLength = 0.3, headRadius = 0.2) {
    const group = new Group()

    // Shaft
    const shaftGeometry = new CylinderGeometry(shaftRadius, shaftRadius, length - headLength, 8)
    const shaftMaterial = new MeshStandardMaterial({ color })
    const shaft = new Mesh(shaftGeometry, shaftMaterial)
    shaft.castShadow = true
    shaft.position.y = (length - headLength) / 2
    group.add(shaft)

    // Head
    const headGeometry = new ConeGeometry(headRadius, headLength, 8)
    const headMaterial = new MeshStandardMaterial({ color })
    const head = new Mesh(headGeometry, headMaterial)
    head.castShadow = true
    head.position.y = length - (headLength / 2)
    group.add(head)

    // Orient arrow
    const up = new Vector3(0, 1, 0)
    const quaternion = new Quaternion().setFromUnitVectors(up, direction.clone().normalize())
    group.quaternion.copy(quaternion)

    group.position.copy(origin)

    return group
}
const xArrow = createArrow(
    new Vector3(1, 0, 0),
    new Vector3(0, 0.5, 0),
    2,
    0xff0000
)
const yArrow = createArrow(
    new Vector3(0, 1, 0),
    new Vector3(0, 0.5, 0),
    2,
    0x00ff00
)
const zArrow = createArrow(
    new Vector3(0, 0, 1),
    new Vector3(0, 0.5, 0),
    2,
    0x0000ff
)

// Local test coordinates and zoom
const lat = 51.4449
const lng = 7.2657
const zoom = 19

const texture = ref(null)
const shadowTex = ref(null)

function long2tile(lon, zoom) {
    return Math.floor(((lon + 180) / 360) * Math.pow(2, zoom))
}

function lat2tile(lat, zoom) {
    const rad = (lat * Math.PI) / 180
    return Math.floor(
        ((1 - Math.log(Math.tan(rad) + 1 / Math.cos(rad)) / Math.PI) / 2) * Math.pow(2, zoom)
    )
}

function getTileUrl(x, y, z) {
    return `https://tile.openstreetmap.org/${z}/${x}/${y}.png`
}

function loadTexture(lat, lng, zoom) {
    const x = long2tile(lng, zoom)
    const y = lat2tile(lat, zoom)
    const url = getTileUrl(x, y, zoom)

    const loader = new TextureLoader()
    loader.load(
        url,
        (tex) => {
            const img = tex.image
            const canvas = document.createElement('canvas')
            canvas.width = img.width
            canvas.height = img.height
            const ctx = canvas.getContext('2d')


            // Draw original image
            ctx.drawImage(img, 0, 0)

            // Get pixel data
            const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height)
            const data = imageData.data

            const contrast = 1.1 // 1 = no change, >1 = higher contrast, <1 = lower contrast

            for (let i = 0; i < data.length; i += 4) {
                for (let c = 0; c < 3; c++) {
                    // Normalize to [0,1]
                    let normalized = data[i + c] / 255
                    // Apply contrast formula centered on 0.5
                    normalized = ((normalized - 0.5) * contrast) + 0.5
                    // Clamp and convert back to [0,255]
                    data[i + c] = Math.min(255, Math.max(0, normalized * 255))
                }
            }

            ctx.putImageData(imageData, 0, 0)

            const adjustedTexture = new CanvasTexture(canvas)
            adjustedTexture.needsUpdate = true

            adjustedTexture.generateMipmaps = false
            adjustedTexture.wrapS = RepeatWrapping
            adjustedTexture.wrapT = RepeatWrapping
            adjustedTexture.repeat.set(2, 2)

            texture.value = adjustedTexture
        },
        undefined,
        (err) => {
            console.error('Error loading tile texture:', err)
        }
    )
}

function loadShadowTexture() {

    const loader = new TextureLoader()
    loader.load(
        shadow,
        (tex) => {
            tex.wrapS = RepeatWrapping
            tex.wrapT = RepeatWrapping
            shadowTex.value = tex
        },
        undefined,
        (err) => {
            console.error('Error loading tile texture:', err)
        }
    )
}

onMounted(() => {
    loadTexture(lat, lng, zoom)
    loadShadowTexture()
})

const orbitRef = ref(null)

watch(orbitRef, (newVal) => {
    if (newVal) {
        const controls = newVal.controls
    }
})


// --- FPS throttle ---
const canvasRef = ref(null)
const targetFPS = 2
const frameInterval = 1000 / targetFPS
let lastTime = performance.now()

function animate(time) {
    const delta = time - lastTime
    if (delta >= frameInterval) {
        canvasRef.value?.renderer?.render(canvasRef.value.scene, canvasRef.value.camera)
        lastTime = time - (delta % frameInterval)
    }
    requestAnimationFrame(animate)
}

onMounted(() => {
    requestAnimationFrame(animate)
})


</script>

<template>
    <div class="frame">
        <MDBRow class="view-controls flex-column gap-2">
            <MDBCol>
                <OhVueIcon name="hi-arrows-expand" scale="1.5" class="input-layer expand-button" />
            </MDBCol>
            <MDBCol>
                <OhVueIcon name="bi-camera-video-fill" scale="1.5" class="input-layer reset-view" />
            </MDBCol>
        </MDBRow>

        <TresCanvas :dpr="2" :alpha="true" :shadows="true" class="rounded" power-preference="low-power">

            <OrbitControls :ref="orbitRef"></OrbitControls>

            <!-- Camera -->
            <TresPerspectiveCamera :position="[3.6, 3.1, 3.6]" :look-at="[0, 0, 0]" />

            <!-- Lights -->
            <TresDirectionalLight :position="[3, 5, -2]" color="#ffffff" :intensity="1.0" :cast-shadow="true"
                :shadow-mapSize="[512, 512]" />
            <TresAmbientLight :intensity="1" />

            <!-- Box on circle -->
            <TresMesh :position="[0, 0.5, 0]" :cast-shadow="true" :receive-shadow="true">
                <TresBoxGeometry :args="[1, 1, 1]" />
                <TresMeshStandardMaterial color="tomato" />
            </TresMesh>

            <!-- Flat Circle with map tile texture -->
            <TresMesh :rotation="[-Math.PI / 2, 0, Math.PI / 4.0]" :receive-shadow="true">
                <TresCircleGeometry :args="[3, 64]" />
                <MeshPhongMaterial v-if="texture" :map="texture" :roughness="1.0" :color="[1.0, 1.0, 1.0]" />
            </TresMesh>

            <!-- Axes arrows -->
            <primitive :object="xArrow" :cast-shadow="true" />
            <primitive :object="yArrow" :cast-shadow="true" />
            <primitive :object="zArrow" :cast-shadow="true" />
        </TresCanvas>
    </div>
</template>


<style scoped>
.rounded-canvas {
    border-radius: 15px;
}

.input-layer {
    background-color: rgba(255, 255, 255, 0.350);
    border-radius: 8px;
    border: 1px rgba(126, 126, 126, 0.178) solid;
}

.view-controls {
    position: absolute;
    right: 0px;
    margin: 3.5px -7.5px;
    z-index: 1;
}

.reset-view-button:hover {
    transform: scale(1.2);
}

.reset-view {
    transition: all 0.3s ease;
    cursor: pointer;
}

.expand-button:hover {
    transform: scale(1.2);
}

.expand-button {
    transition: all 0.3s ease;
    cursor: pointer;
}

.frame-big {
    position: relative;
    width: 100%;
    height: 30vh;
    background: transparent;
}

.frame {
    position: relative;
    width: 200px;
    height: 128px;
    background: transparent;
}
</style>
