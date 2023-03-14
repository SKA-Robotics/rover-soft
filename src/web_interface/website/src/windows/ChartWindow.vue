<script setup>
import { useRosStore } from '@/stores'
import { defineProps, watch, ref, onBeforeUnmount } from 'vue'
import { Topic } from 'roslib'
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
const seriesConfig = ref([])
const refreshingTimer = ref(null)
const chartTitle = ref('')
const yAxisTitle = ref('')

watch(props, () => {
    seriesConfig.value.forEach((serie, index) => {
        if (serie.listener) serie.listener.unsubscribe()
        data.value.datasets[index].data.length = 0
    })
    data.value.datasets.length = 0

    seriesConfig.value = JSON.parse(
        JSON.stringify(props.extraConfig.series)
    ).slice(0, props.extraConfig.seriesNumber)

    chartTitle.value = props.extraConfig.chartTitle
    yAxisTitle.value = props.extraConfig.yAxisTitle

    rosStore.ws.getTopics((result) => {
        setChartOptions()
        seriesConfig.value.forEach((serie, i) => {
            result.topics.forEach((name, j) => {
                if (name == serie.topicName) serie.messageType = result.types[j]
            })
            setListener(serie, i)
        })
    })

    if (refreshingTimer.value != null) clearInterval(refreshingTimer.value)
    refreshingTimer.value = setInterval(() => {
        rerenderChart()
    }, 1000 / props.extraConfig.refreshingFrequency)
})

// updating chart with ROS topic data
const startTime = ref(0)

function setListener(serie, index) {
    // console.log(`Listner got message with type: ${serie.messageType}`)
    setSerieOptions(serie, index)
    const keys = serie.messageProperty.split('/')

    serie.listener = new Topic({
        ros: rosStore.ws,
        name: serie.topicName,
        messageType: serie.messageType,
    })

    serie.listener.subscribe((msg) => {
        let prop = msg
        try {
            keys.forEach((key) => {
                if (key != '') prop = prop[key]
            })
        } catch (error) {
            // console.log(`Message: ${msg} results an error occurance. ${error}`)
            return
        }
        let point = {
            x: new Date().getTime() / 1000 - startTime.value,
            y: parseFloat(prop),
        }
        // console.log(point)
        if (!isNaN(point.y)) data.value.datasets[index].data.push(point)
    })
}

// changing configuration of the entire chart and each serie
function setChartOptions() {
    if (seriesConfig.value.length == 1) {
        options.value.scales.y.title.display = true
        options.value.plugins.legend.display = false
        options.value.plugins.title.display = true
    } else {
        options.value.scales.y.title.display = yAxisTitle.value
        options.value.scales.y.title.text = yAxisTitle.value
        options.value.plugins.legend.display = true
        options.value.plugins.title.display = chartTitle.value
        options.value.plugins.title.text = chartTitle.value
    }
    const currentDate = new Date()
    startTime.value = currentDate.getTime() / 1000
    let hours = currentDate.getHours().toFixed(0)
    if (hours.length < 2) hours = '0' + hours
    let minutes = currentDate.getMinutes().toFixed(0)
    if (minutes.length < 2) minutes = '0' + minutes
    let seconds = currentDate.getSeconds().toFixed(0)
    if (seconds.length < 2) seconds = '0' + seconds
    options.value.scales.x.title.text = `Time [s] starting at ${hours}:${minutes}:${seconds}`
}

function setSerieOptions(serie, index) {
    if (!serie.messageProperty) serie.messageProperty = ''
    serie.messageProperty.trim()

    if (seriesConfig.value.length == 1) {
        options.value.scales.y.title.text = yAxisTitle.value
            ? yAxisTitle.value
            : serie.messageProperty
        options.value.plugins.title.text = chartTitle.value
            ? chartTitle.value
            : serie.topicName
    }

    const angle = (index * 2 * Math.PI) / seriesConfig.value.length
    const r = 170 + 85 * Math.cos(angle)
    const g = 170 + 85 * Math.cos(angle - (2 * Math.PI) / 3)
    const b = 170 + 85 * Math.cos(angle - (4 * Math.PI) / 3)

    data.value.datasets.push({
        label: serie.label
            ? serie.label
            : serie.topicName + '/' + serie.messageProperty,
        fill: false,
        borderColor: `rgb(${r}, ${g}, ${b})`,
        backgroundColor: `rgb(${r}, ${g}, ${b})`,
        data: [],
    })
}

// chart rerfreshing using key changing technique
const chartKey = ref(0)

function rerenderChart() {
    chartKey.value += 1
}

// setting initial configuration
const data = ref({
    datasets: [
        {
            label: 'ROS value [u]',
            fill: false,
            data: [],
        },
    ],
})
const options = ref({
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
                font: {
                    weight: 'bold',
                },
            },
            beginAtZero: true,
        },
        y: {
            display: true,
            title: {
                display: true,
                text: 'ROS value [u]',
                font: {
                    weight: 'bold',
                },
            },
        },
    },
    plugins: {
        legend: {
            display: false,
            position: 'top',
            labels: {
                font: {
                    weight: 'bold',
                },
            },
        },
        title: {
            display: false,
            font: {
                size: 16,
                weight: 'bold',
            },
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
})

onBeforeUnmount(() => {
    seriesConfig.value.forEach((serie, index) => {
        if (serie.listener) serie.listener.unsubscribe()
        delete data.value.datasets[index].data
    })
    if (refreshingTimer.value != null) clearInterval(refreshingTimer.value)
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
