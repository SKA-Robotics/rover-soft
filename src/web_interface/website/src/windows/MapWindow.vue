<script setup>
import { defineProps, ref, onMounted, computed, onBeforeUnmount } from 'vue'

const props = defineProps(['windowDimensions', 'extraConfig'])

const map = ref(null)
const image = ref(null)

const interval = ref(null)
const isMoving = ref(false)
const moveOffset = ref({
    x: 0,
    y: 0,
})

const overlaySize = ref(50)

const scale = ref(null)
const angle = ref(0)
const viewCenterUnit = ref({
    x: 0,
    y: 0,
})

const direction = ref(0)
const directionTriangle = computed(() => {
    return {
        x1: (-0.6 * overlaySize.value) / scale.value,
        y1: (0 * overlaySize.value) / scale.value,
        x2: (0 * overlaySize.value) / scale.value,
        y2: (-1.8 * overlaySize.value) / scale.value,
        x3: (0.6 * overlaySize.value) / scale.value,
        y3: (0 * overlaySize.value) / scale.value,
    }
})
const imageUnit = ref({
    x: 0,
    y: 0,
    width: 6,
    height: 6,
})
const currentPosUnit = ref({
    x: 1,
    y: 1,
})

const windowCenterPixel = computed(() => {
    return {
        x: props.windowDimensions.width / 2,
        y: props.windowDimensions.height / 2,
    }
})

/*const windowCenterUnit = computed(() => {
    return {
        y: props.windowDimensions.height / (2 * scale.value),
        x: props.windowDimensions.width / (2 * scale.value),
    }
})*/

onMounted(() => {
    let img = new Image()
    img.src = '/maps/mapa.jpg'

    img.onload = () => {
        image.value = img
        resetView()
    }

    interval.value = setInterval(draw, 10)
})

onBeforeUnmount(() => {
    clearInterval(interval.value)
})

//TODO: add a button to reset
const resetView = () => {
    scale.value = props.windowDimensions.height / imageUnit.value.height
    viewCenterUnit.value.x = imageUnit.value.x + imageUnit.value.width / 2
    viewCenterUnit.value.y = imageUnit.value.y + imageUnit.value.height / 2
}

const transform = () => {
    let ctx = map.value.getContext('2d')
    ctx.translate(windowCenterPixel.value.x, windowCenterPixel.value.y)
    ctx.scale(scale.value, scale.value)
    ctx.rotate((angle.value * Math.PI) / 180)
    ctx.translate(-viewCenterUnit.value.x, -viewCenterUnit.value.y)
}

const draw = () => {
    let ctx = map.value.getContext('2d')
    ctx.resetTransform()
    ctx.clearRect(0, 0, map.value.width, map.value.height)
    //rysowac okienka
    transform()

    drawMap()
    drawPosition()

    drawGrid()
}

const drawMap = () => {
    if (map.value && image.value) {
        let ctx = map.value.getContext('2d')
        ctx.drawImage(
            image.value,
            imageUnit.value.x,
            imageUnit.value.y,
            imageUnit.value.width,
            imageUnit.value.height
        )
    } else {
        console.log('brak obrazka')
    }
}
const drawPosition = () => {
    let ctx = map.value.getContext('2d')
    ctx.beginPath()
    ctx.lineWidth = overlaySize.value / scale.value
    ctx.strokeStyle = 'red'
    ctx.fillStyle = 'red'
    ctx.arc(
        currentPosUnit.value.x,
        currentPosUnit.value.y,
        overlaySize.value / scale.value,
        0,
        Math.PI * 2
    )
    ctx.fill()
    ctx.stroke()
    drawDirection()
}

const drawDirection = () => {
    let ctx = map.value.getContext('2d')
    ctx.beginPath()
    ctx.strokeStyle = 'black'
    ctx.fillStyle = 'black'

    const r1 = Math.hypot(
        directionTriangle.value.x1,
        directionTriangle.value.y1
    )
    const r2 = Math.hypot(
        directionTriangle.value.x2,
        directionTriangle.value.y2
    )
    const r3 = Math.hypot(
        directionTriangle.value.x3,
        directionTriangle.value.y3
    )
    const phi1 = Math.atan2(
        directionTriangle.value.x1,
        directionTriangle.value.y1
    )
    const phi2 = Math.atan2(
        directionTriangle.value.x2,
        directionTriangle.value.y2
    )
    const phi3 = Math.atan2(
        directionTriangle.value.x3,
        directionTriangle.value.y3
    )

    console.log(
        currentPosUnit.value.y +
            r1 * Math.sin((direction.value * Math.PI) / 180)
    )

    //przez wektor r dlugosc r nie rozroznia x1 od x3 i y1 od y3

    ctx.moveTo(
        currentPosUnit.value.x +
            r1 * Math.cos((direction.value * Math.PI) / 180 + phi1),
        currentPosUnit.value.y +
            r1 * Math.sin((direction.value * Math.PI) / 180 + phi1)
    )
    ctx.lineTo(
        currentPosUnit.value.x +
            r2 * Math.cos((direction.value * Math.PI) / 180 + phi2),
        currentPosUnit.value.y +
            r2 * Math.sin((direction.value * Math.PI) / 180 + phi2)
    )
    ctx.lineTo(
        currentPosUnit.value.x +
            r3 * Math.cos((direction.value * Math.PI) / 180 + phi3),
        currentPosUnit.value.y +
            r3 * Math.sin((direction.value * Math.PI) / 180 + phi3)
    )
    ctx.closePath()

    ctx.fill()
    ctx.stroke()
}
/*
const drawDirection = () => {
    let x1 = currentPosUnit.value.x + directionTriangle.value.x1
    let y1 = currentPosUnit.value.y + directionTriangle.value.y1
    let x2 = currentPosUnit.value.x + directionTriangle.value.x2
    let y2 = currentPosUnit.value.y + directionTriangle.value.y2
    let x3 = currentPosUnit.value.x + directionTriangle.value.x3
    let y3 = currentPosUnit.value.y + directionTriangle.value.y3

    let r1 = Math.sqrt(
        Math.pow(directionTriangle.value.x1, 2) +
            Math.pow(directionTriangle.value.y1, 2)
    )
    let r2 = Math.sqrt(
        Math.pow(directionTriangle.value.x2, 2) +
            Math.pow(directionTriangle.value.y2, 2)
    )
    let r3 = Math.sqrt(
        Math.pow(directionTriangle.value.x3, 2) +
            Math.pow(directionTriangle.value.y3, 2)
    )

    let ctx = map.value.getContext('2d')
    ctx.beginPath()
    ctx.strokeStyle = 'green'
    ctx.fillStyle = 'green'
    ctx.lineWidth = overlaySize.value / scale.value

    ctx.lineTo(
        currentPosUnit.value.x +
            r1 * Math.cos(((direction.value + Math.PI / 4) * Math.PI) / 180),
        currentPosUnit.value.y +
            r1 * Math.sin(((direction.value + Math.PI / 4) * Math.PI) / 180)
    )
    ctx.lineTo(
        currentPosUnit.value.x +
            r2 * Math.cos((direction.value * Math.PI) / 180),
        currentPosUnit.value.y +
            r2 * Math.sin((direction.value * Math.PI) / 180)
    )
    ctx.lineTo(
        currentPosUnit.value.x +
            r3 * Math.cos(((direction.value - Math.PI / 4) * Math.PI) / 180),
        currentPosUnit.value.y +
            r3 * Math.sin(((direction.value - Math.PI / 4) * Math.PI) / 180)
    )
    ctx.lineTo(
        currentPosUnit.value.x +
            r1 * Math.cos((direction.value * Math.PI) / 180),
        currentPosUnit.value.y +
            r1 * Math.sin((direction.value * Math.PI) / 180)
    )

    console.log(r1 * Math.cos((direction.value * Math.PI) / 180))

    ctx.fill()
    ctx.stroke()
}*/

const drawGrid = () => {
    let x = (imageUnit.value.width * 3) / 2
    let h = (imageUnit.value.height * 3) / 2

    let ctx = map.value.getContext('2d')
    ctx.lineWidth = 0.02

    ctx.beginPath()
    for (var i = -x; i <= x; i++) {
        ctx.moveTo(
            i + imageUnit.value.height / 2,
            -h + imageUnit.value.width / 2
        )
        ctx.lineTo(
            i + imageUnit.value.height / 2,
            h + imageUnit.value.width / 2
        )
    }

    for (var j = -h; j <= h; j++) {
        ctx.moveTo(
            -x + imageUnit.value.width / 2,
            j + imageUnit.value.height / 2
        )
        ctx.lineTo(
            x + imageUnit.value.width / 2,
            j + imageUnit.value.height / 2
        )
    }

    ctx.strokeStyle = 'rgba(0, 0, 0, 0.5)'

    ctx.stroke()
}

const zoom = (e) => {
    if (e.deltaY > 0) {
        return (scale.value *= 0.9)
    } else if (e.deltaY < 0) {
        return (scale.value *= 1.1)
    }
}

const move = (e) => {
    if (isMoving.value) {
        let deltaX = e.offsetX - moveOffset.value.x
        let deltaY = e.offsetY - moveOffset.value.y
        let r = Math.hypot(deltaX, deltaY)
        let phi = Math.atan2(deltaY, deltaX)

        let deltaXRad = r * Math.cos((-angle.value * Math.PI) / 180 + phi)
        let deltaYRad = r * Math.sin((-angle.value * Math.PI) / 180 + phi)

        viewCenterUnit.value.x -= deltaXRad / scale.value
        viewCenterUnit.value.y -= deltaYRad / scale.value

        moveOffset.value.x = e.offsetX
        moveOffset.value.y = e.offsetY
    }
}

const mouseDown = (e) => {
    moveOffset.value.x = e.offsetX
    moveOffset.value.y = e.offsetY
    isMoving.value = true
}

const mouseUp = () => {
    isMoving.value = false
}
</script>
<template>
    <div
        @wheel="zoom"
        @mousemove="move"
        @mousedown="mouseDown"
        @mouseup="mouseUp"
        :style="{
            display: 'flex',
            'align-items': 'center',
            'justify-content': 'center',
            width: props.windowDimensions.width + 'px',
            height: props.windowDimensions.height + 'px',
            overflow: 'hidden',
            position: 'fixed',
        }"
    >
        <canvas
            :width="props.windowDimensions.width"
            :height="props.windowDimensions.height"
            ref="map"
        ></canvas>
    </div>
</template>
