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

// getting settings update
const config = ref(JSON.parse(JSON.stringify(props.extraConfig)))
watch(props, () => {
    config.value = ref(JSON.parse(JSON.stringify(props.extraConfig)))
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
})
const topicName = computed(() => {
    return config.value.topicName
})
const messageType = ref(null)
const title = computed(() => {
    return `Topic name: ${topicName.value}, Message type: ${messageType.value}`
})

// updating chart with ROS topic data
const startTime = ref(0)
const listener = ref(null)
function setListener() {
    let callback = () => {
        // defined conditionally below
    }
    console.log(`Listner got message with type: ${messageType.value}`)

    switch (messageType.value) {
        case 'geometry_msgs/Twist':
            callback = (msg) => {
                let point = {
                    x: new Date().getTime() / 1000 - startTime.value,
                    y: msg.linear.x,
                }
                // console.log(point)
                data.value.datasets[0].data.push(point)
                rerenderChart()
            }
            break
        case 'sensor_msgs/JointState':
            callback = (msg) => {
                let point = {
                    x:
                        msg.header.stamp.secs +
                        1e-9 * msg.header.stamp.nsecs -
                        startTime.value,
                    y: msg.effort[0],
                }
                // console.log(point)
                data.value.datasets[0].data.push(point)
                rerenderChart()
            }
            break
        default:
            console.log('Invalid message type!')
            return
    }

    if (listener.value) listener.value.clear()
    startTime.value = new Date().getTime() / 1000
    listener.value = new Topic({
        ros: rosStore.ws,
        name: topicName,
        messageType: messageType,
    })
    listener.value.subscribe(callback)
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
