<script setup>
import { useRosStore } from '@/stores'
import {
    defineProps,
    onBeforeMount,
    watch,
    ref,
    computed,
    shallowRef,
} from 'vue'
import { Service, ServiceRequest, Topic } from 'roslib'
import { Scatter } from 'vue-chartjs'
import {
    LinearScale,
    PointElement,
    LineElement,
    Title,
    Tooltip,
    Legend,
    Chart,
} from 'chart.js'
Chart.register(LinearScale, PointElement, LineElement, Title, Tooltip, Legend)

const props = defineProps(['extraConfig'])

const rosStore = useRosStore()

// getting config update
const config = ref(JSON.parse(JSON.stringify(props.extraConfig)))
watch(props, () => {
    config.value = ref(JSON.parse(JSON.stringify(props.extraConfig)))
    topicName.value = config.value.topicName
    messageProperty.value = config.value.messageProperty

    const topicsClient = new Service({
        ros: rosStore.ws,
        name: '/rosapi/topics',
        serviceType: 'rosapi/Topics',
    })

    const request = new ServiceRequest()

    topicsClient.callService(request, (result) => {
        // console.log(result)
        result.topics.forEach((name, index) => {
            if (name == topicName.value) messageType.value = result.types[index]
        })
        setListener()
    })

    if (refreshingTimer.value != null) clearInterval(refreshingTimer.value)
    refreshingTimer.value = setInterval(() => {
        rerenderChart()
    }, 1000 / config.value.refreshingFrequency)
})
const topicName = ref(null)
const messageType = ref(null)
const messageProperty = ref(null)
const title = computed(() => {
    return `Topic name: ${topicName.value},  Message type: ${messageType.value}`
})
const refreshingTimer = ref(null)

// updating chart with ROS topic data
const startTime = ref(0)
const listener = ref(null)
function setListener() {
    // console.log(`Listner got message with type: ${messageType.value}`)
    if (!messageProperty.value) messageProperty.value = 'linear/x'
    messageProperty.value.trim()
    const keys = messageProperty.value.split('/')
    options.value.scales.y.title.text = messageProperty.value
    delete data.value.datasets[0].data
    data.value.datasets[0].data = []
    // console.log(messageProperty.value)

    if (listener.value) listener.value.unsubscribe()
    const currentDate = new Date()
    startTime.value = currentDate.getTime() / 1000
    let hours = currentDate.getHours().toFixed(0)
    if (hours.length < 2) hours = '0' + hours
    let minutes = currentDate.getMinutes().toFixed(0)
    if (minutes.length < 2) minutes = '0' + minutes
    let seconds = currentDate.getSeconds().toFixed(0)
    if (seconds.length < 2) seconds = '0' + seconds
    options.value.scales.x.title.text = `Time [s] starting at ${hours}:${minutes}:${seconds}`

    listener.value = new Topic({
        ros: rosStore.ws,
        name: topicName,
        messageType: messageType,
    })
    listener.value.subscribe((msg) => {
        keys.forEach((key) => {
            if (key != '') msg = msg[key]
        })
        let point = {
            x: new Date().getTime() / 1000 - startTime.value,
            y: parseFloat(msg),
        }
        // console.log(point)
        if (!isNaN(point.y)) data.value.datasets[0].data.push(point)
    })
}

// chart rerfreshing using key changing technique
const chartKey = ref(0)
function rerenderChart() {
    chartKey.value += 1
}

// setting initial configuration
const data = ref(null)
const options = ref(null)
onBeforeMount(() => {
    data.value = {
        datasets: [
            {
                label: 'ROS value [u]',
                fill: false,
                borderColor: '#f87979',
                backgroundColor: '#f87979',
                data: shallowRef([{ x: Number, y: Number }]),
            },
        ],
    }
    options.value = {
        responsive: true,
        maintainAspectRatio: false,
        showLine: true,
        animation: false,
        scales: {
            x: {
                display: true,
                title: {
                    display: true,
                    text: 'Time [s]',
                },
                beginAtZero: true,
            },
            y: {
                display: true,
                title: {
                    display: true,
                    text: 'ROS value [u]',
                },
            },
        },
        plugins: {
            legend: {
                display: false,
            },
            title: {
                display: true,
                text: title,
                padding: {
                    top: 0,
                    bottom: 10,
                },
            },
        },
        elements: {
            point: {
                radius: 1,
                borderColor: '#ff0000',
                backgroundColor: '#ff0000',
            },
            line: {
                borderColor: '#ff0000',
                backgroundColor: '#ff0000',
            },
        },
    }
})
</script>

<template>
    <div>
        <Scatter
            class="chart"
            :key="chartKey"
            :data="data"
            :options="options"
        />
    </div>
</template>

<style scoped>
div {
    display: flex;
    width: 100%;
    align-items: center;
    justify-content: center;
}
.chart {
    padding: 2%;
}
</style>
