<script setup>
import { defineProps, ref, onMounted, computed, onBeforeUnmount } from 'vue'

const props = defineProps(['windowDimensions', 'extraConfig'])

const map = ref(null)
const image = ref(null)
const interval = ref(null)
const isMoving = ref(false)

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
const currentScale = ref(null)

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
        currentScale.value = imageSizeUnits.value
    }
    interval.value = setInterval(draw, 100)
})

onBeforeUnmount(() => {
    clearInterval(interval.value)
})

const zoom = () => {
    if (event.deltaY > 0) {
        return (currentScale.value *= 1.1)
    } else if (event.deltaY < 0) {
        return (currentScale.value *= 0.9)
    }
}

const unitsToPx = computed(() => {
    let change = {
        y: imgDimPx.value.height / currentScale.value,
        x: imgDimPx.value.width / currentScale.value,
    }

    return change
})

const positionTranslation = (xUnit, yUnit) => {
    let windowCenter = {
        x: props.windowDimensions.width / 2,
        y: props.windowDimensions.height / 2,
    }

    let translation = {
        x: windowCenter.x + (centerUnit.value.x + xUnit) * unitsToPx.value.x,
        y: windowCenter.y + (centerUnit.value.y - yUnit) * unitsToPx.value.y,
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

const draw = () => {
    drawMap()
    drawPosition()
}

const drawMap = () => {
    if (map.value && image.value) {
        let ctx = map.value.getContext('2d')
        ctx.drawImage(
            image.value,
            windowStart.value.x,
            windowStart.value.y,
            view.value.width,
            view.value.height
        )
    } else {
        console.log('brak obrazka')
    }
}

const drawPosition = () => {
    let ctx = map.value.getContext('2d')
    ctx.beginPath()
    ctx.lineWidth = 3
    ctx.strokeStyle = 'red'
    ctx.arc(roverOnMapPos.value.x, roverOnMapPos.value.y, 1, 1, Math.PI * 2)
    ctx.stroke()
}

const move = () => {
    if (isMoving) {
        centerUnit.value.x = offset.value.x / unitsToPx.value.x
        centerUnit.value.y = offset.value.y / unitsToPx.value.y
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
