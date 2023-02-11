<script setup>
import { defineProps, ref, onMounted, computed, onBeforeUnmount } from 'vue'

const props = defineProps(['windowDimensions', 'extraConfig'])
const imgDim = ref({
    width: null,
    height: null,
})

let currentPos = ref({
    x: 100,
    y: 100,
})

/*const roverPosition = computed(() => {
    let mapHeight = dimensions.maxHeight
    let mapWidth = dimensions.maxWidth


})*/

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

const map = ref(null)
const image = ref(null)
const interval = ref(null)
const posInterval = ref(null)

const draw = () => {
    if (map.value && image.value) {
        //console.log('w' + dimensions.value.width)
        //console.log('h' + dimensions.value.height)
        let ctx = map.value.getContext('2d')
        ctx.drawImage(
            image.value,
            0,
            0,
            dimensions.value.width,
            dimensions.value.height
        )
    } else {
        console.log('brak obrazka')
    }
}

onMounted(() => {
    let img = new Image()
    img.src = '/maps/mapa.jpg'

    img.onload = () => {
        image.value = img
        imgDim.value.height = img.height
        imgDim.value.width = img.width
    }

    interval.value = setInterval(draw, 10)
    posInterval.value = setInterval(drawPosition, 10)
})

onBeforeUnmount(() => {
    clearInterval(interval.value)
})

let drawPosition = () => {
    clearInterval(posInterval)
    let ctx = map.value.getContext('2d')
    console.log('kropka x' + currentPos.value.x)
    console.log('kropka y' + currentPos.value.y)
    ctx.beginPath()
    ctx.lineWidth = 3
    ctx.strokeStyle = 'red'
    ctx.arc(currentPos.value.x, currentPos.value.y, 1, 1, Math.PI * 2)
    ctx.stroke()
}
</script>
<template>
    <div
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
