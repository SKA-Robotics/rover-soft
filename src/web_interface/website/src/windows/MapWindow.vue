<script setup>
import { defineProps, ref, onMounted, computed, onBeforeUnmount } from 'vue'

const props = defineProps(['windowDimensions', 'extraConfig'])

const map = ref(null)
const image = ref(null)
const interval = ref(null)
const isMoving = ref(false)
const angle = ref(45)

const offset = ref({
    x: 0,
    y: 0,
})

const imgDimPx = ref({
    width: null,
    height: null,
})

const currentPosUnit = ref({
    x: 1,
    y: 1,
})

const centerUnit = ref({
    x: 0,
    y: 0,
})

const imageCornerUnit = ref({
    x: -3,
    y: 3,
})

const imageSizeUnits = ref(6)
const scale = ref(null)

const roverOnMapPos = computed(() => {
    let roverPos = {
        x: positionTranslation(currentPosUnit.value.x, currentPosUnit.value.y)
            .x,
        y: positionTranslation(currentPosUnit.value.x, currentPosUnit.value.y)
            .y,
    }
    return roverPos
})

onMounted(() => {
    let img = new Image()
    img.src = '/maps/mapa.jpg'

    img.onload = () => {
        image.value = img
        imgDimPx.value.height = img.height
        imgDimPx.value.width = img.width
        scale.value = imageSizeUnits.value
    }
    interval.value = setInterval(draw, 10)
    //TODO: consider otation setInterval :p
    rotate(90)
})

onBeforeUnmount(() => {
    clearInterval(interval.value)
})

const zoom = () => {
    if (event.deltaY > 0) {
        return (scale.value *= 1.1)
    } else if (event.deltaY < 0) {
        return (scale.value *= 0.9)
    }
}

const unitsToPx = computed(() => {
    let change = {
        y: imgDimPx.value.height / scale.value,
        x: imgDimPx.value.width / scale.value,
    }

    return change
})
/*
const unitsToPixelTransform = computed((xUnit, yUnit) => {
    let pixels = {
        x: xUnit * unitsToPx.value.x,
        y: yUnit * unitsToPx.value.x,
    }

    return pixels
})

const pixelToUnitsTransform = computed((x, y) => {
    let units = {
        x: x / unitsToPx.value.x,
        y: y / unitsToPx.value.y,
    }

    return units
})

const cartesianToCircular = computed((xUnit, yUnit) => {
    //cartesian to circular in units, from center pos in niuts(0, 0)
    let r = Math.sqrt(Math.pow(xUnit, 2) + Math.pow(yUnit, 2))

    let position = {
        x: r * Math.cos(angle.value),
        y: r * Math.sin(angle.value),
    }

    return position
})
*/
/*const circularToCartesian = computed((r, theta) => {
    let position = {
        x:
    }
})*/

const centerPixel = computed(() => {
    let windowCenter = {
        x: props.windowDimensions.width / 2,
        y: props.windowDimensions.height / 2,
    }

    return windowCenter
})

const positionTranslation = (xUnit, yUnit) => {
    let translation = {
        x:
            centerPixel.value.x +
            (centerUnit.value.x + xUnit) * unitsToPx.value.x,
        y:
            centerPixel.value.y +
            (centerUnit.value.y - yUnit) * unitsToPx.value.y,
    }

    return translation
}

const windowStart = computed(() => {
    let currentViewCorner = {
        x: positionTranslation(imageCornerUnit.value.x, imageCornerUnit.value.y)
            .x,
        y: positionTranslation(imageCornerUnit.value.x, imageCornerUnit.value.y)
            .y,
    }

    return currentViewCorner
})

const view = computed(() => {
    let window = {
        x: centerUnit.value.x,
        y: centerUnit.value.y,
        height: unitsToPx.value.y * imageSizeUnits.value,
        width: unitsToPx.value.x * imageSizeUnits.value,
    }

    return window
})

const rotate = () => {
    let ctx = map.value.getContext('2d')
    ctx.resetTransform()
    ctx.rotate((angle.value * Math.PI) / 180)
    /*angle.value = newAngle
    image.rotate(angle.value)*/
    //dwa plotna
}

const draw = () => {
    angle.value += 1
    drawMap()
    drawPosition()
    rotate()
}

const drawMap = () => {
    if (map.value && image.value) {
        let ctx = map.value.getContext('2d')
        //ctx.rotate((angle.value * Math.PI) / 180)

        //let width = image.width
        //let height = image.height

        ctx.drawImage(
            image.value,
            windowStart.value.x,
            windowStart.value.y,
            view.value.width,
            view.value.height
        )
        //rotate(45)
    } else {
        console.log('brak obrazka')
    }
}

const drawPosition = () => {
    let ctx = map.value.getContext('2d')
    ctx.beginPath()
    ctx.lineWidth = 3
    ctx.strokeStyle = 'red'
    ctx.arc(roverOnMapPos.value.x, roverOnMapPos.value.y, 1, 1, Math.PI * 2) //to change x and y to consider rotation
    ctx.stroke()
}

const move = (e) => {
    if (isMoving.value) {
        let deltaX = e.offsetX - offset.value.x
        let deltaY = e.offsetY - offset.value.y
        let r = Math.hypot(deltaX, deltaY)
        let phi = Math.atan2(deltaY, deltaX)

        let deltaXRad = r * Math.cos((-angle.value * Math.PI) / 180 + phi)
        let deltaYRad = r * Math.sin((-angle.value * Math.PI) / 180 + phi)

        //centerUnit.value.x += deltaX / unitsToPx.value.x
        centerUnit.value.x += deltaXRad / unitsToPx.value.x
        centerUnit.value.y += deltaYRad / unitsToPx.value.y

        offset.value.x = e.offsetX
        offset.value.y = e.offsetY
    }
}

const mouseDown = (e) => {
    offset.value.x = e.offsetX
    offset.value.y = e.offsetY
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
