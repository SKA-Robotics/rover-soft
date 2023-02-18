<script setup>
import { defineProps, ref, onMounted, computed, onBeforeUnmount } from 'vue'

const props = defineProps(['windowDimensions', 'extraConfig'])

const map = ref(null)
const image = ref(null)
const interval = ref(null)

const imgDim = ref({
    width: null,
    height: null,
})

const currentPos = ref({
    x: 100,
    y: 100,
})

const centerPosition = ref({
    x: 0,
    y: 0,
})

const virtualImageCorner = ref({
    x: -3,
    y: 3,
})

const virtualImagSize = ref(6)
const currentScale = ref(null)

const roverOnMapPos = computed(() => {
    let roverPos = {
        x: currentPos.value.x /* scale.value.x*/, //TODO check if works corrdctly
        y: currentPos.value.y /** scale.value.y*/,
    }
    return roverPos
})

onMounted(() => {
    let img = new Image()
    img.src = '/maps/mapa.jpg'

    img.onload = () => {
        image.value = img
        imgDim.value.height = img.height
        imgDim.value.width = img.width
        currentScale.value = virtualImagSize.value
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

const dimensions = computed(() => {
    const aspectRatio = imgDim.value.width / imgDim.value.height
    let { width, height } = props.windowDimensions

    let maxWidth = {
        width,
        height: width / aspectRatio,
    }
    let maxHeight = {
        width: height * aspectRatio,
        height,
    }

    // choose the one that will fit
    if (height < maxWidth.height) {
        return maxHeight
    } else {
        return maxWidth
    }
})

const unitsToPx = computed(() => {
    let change = {
        y: imgDim.value.height / currentScale.value,
        x: imgDim.value.width / currentScale.value,
    }
    console.log('unitspx' + change.x)
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
            (centerPosition.value.x + virtualImageCorner.value.x) *
                unitsToPx.value.x,
        y:
            windowCenter.y +
            (centerPosition.value.y - virtualImageCorner.value.y) *
                unitsToPx.value.y,
    }

    console.log('srodek' + windowCenter.x)
    console.log('rogscale' + currentScale.value)
    console.log('rog' + currentViewCorner.x)

    return currentViewCorner
})

const view = computed(() => {
    console.log('units' + unitsToPx.value.x)
    let window = {
        x: centerPosition.value.x,
        y: centerPosition.value.y,
        height: unitsToPx.value.y * virtualImagSize.value,
        width: unitsToPx.value.x * virtualImagSize.value,
    }

    return window
})

//dac osobne computed na widok(skala, pozycja)
//dac linie do drugiej przestrzeni(jakis uklad kartezjanski)

const draw = () => {
    drawMap()
    drawPosition()
    console.log('test dim' + dimensions.value.height)
    console.log('test img' + imgDim.value.width)
}

const drawMap = () => {
    if (map.value && image.value) {
        console.log('w' + view.value.width)
        console.log('h' + unitsToPx.value.height)
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

    console.log('kropka x' + currentPos.value.x)
    console.log('kropka y' + currentPos.value.y)
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
