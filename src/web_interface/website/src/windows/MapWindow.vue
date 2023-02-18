<script setup>
import { defineProps, ref, onMounted, computed, onBeforeUnmount } from 'vue'

const props = defineProps(['windowDimensions', 'extraConfig'])

const map = ref(null)
const image = ref(null)
const interval = ref(null)

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
        x:
            (centerUnit.value.x + currentPosUnit.value.x) *
            positionTranslation.value.x,
        y:
            (centerUnit.value.y + currentPosUnit.value.y) *
            positionTranslation.value.y,
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

const positionTranslation = computed(() => {
    let change = {
        y: imgDimPx.value.height / currentScale.value,
        x: imgDimPx.value.width / currentScale.value,
    }
    return change
})

const windowStart = computed(() => {
    let windowCenter = {
        x: props.windowDimensions.width / 2,
        y: props.windowDimensions.height / 2,
    }

    let currentViewCorner = {
        x:
            windowCenter.x +
            (centerUnit.value.x + imageCornerUnit.value.x) *
                positionTranslation.value.x,
        y:
            windowCenter.y +
            (centerUnit.value.y - imageCornerUnit.value.y) *
                positionTranslation.value.y,
    }

    return currentViewCorner
})

const view = computed(() => {
    let window = {
        x: centerUnit.value.x,
        y: centerUnit.value.y,
        height: positionTranslation.value.y * imageSizeUnits.value,
        width: positionTranslation.value.x * imageSizeUnits.value,
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

    console.log('kropka x' + currentPosUnit.value.x)
    console.log('kropka y' + currentPosUnit.value.y)
    console.log('obliczone x' + roverOnMapPos.value.x)
    console.log('obliczone y' + roverOnMapPos.value.y)
    ctx.beginPath()
    ctx.lineWidth = 3
    ctx.strokeStyle = 'red'
    ctx.arc(roverOnMapPos.value.x, roverOnMapPos.value.y, 1, 1, Math.PI * 2)
    ctx.stroke()
}
</script>
<template>
    <div
        @wheel="zoom"
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
