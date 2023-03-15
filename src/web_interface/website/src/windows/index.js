import TestWindow from './TestWindow.vue'
import CameraWindow from './CameraWindow.vue'
import ChartWindow from './ChartWindow.vue'
import { useGstreamerStore, useRosStore } from '@/stores'
import { ref } from 'vue'

export default {
    testWindow: {
        typeName: 'Test Window',
        component: TestWindow,
        configOptions: {
            videoSource: {
                name: 'Video Source',
                type: 'select',
                possibleValues: () => {
                    const gstreamerStore = useGstreamerStore()
                    return Object.keys(gstreamerStore.peers)
                },
            },
            textTest: {
                name: 'Text test',
                type: 'text',
            },
            numberTest: {
                name: 'Number test',
                type: 'number',
            },
            rangeTest: {
                name: 'Range test',
                type: 'range',
                range: () => ({ min: -0.2, max: 5, step: 0.1 }),
            },
            aaaa: {
                name: 'AAAAA',
                type: 'range',
                range: () => ({ min: -0.2, max: 5, step: 0.1 }),
            },
            bbbb: {
                name: 'BBBBBB',
                type: 'boolean',
            },
        },
        icon: 'mdi-camera',
    },
    cameraWindow: {
        typeName: 'Camera Window',
        component: CameraWindow,
        configOptions: {
            videoSource: {
                name: 'Video Source',
                type: 'select',
                possibleValues: () => {
                    const gstreamerStore = useGstreamerStore()
                    return Object.keys(gstreamerStore.peers)
                },
            },
        },
        icon: 'mdi-camera',
    },
    chartWindow: {
        typeName: 'Chart Window',
        component: ChartWindow,
        configOptions: {
            chartTitle: {
                name: 'Chart Title',
                type: 'text',
            },
            yAxisTitle: {
                name: 'Y-axis Title',
                type: 'text',
            },
            refreshingFrequency: {
                name: 'Refreshing Frequency [Hz]',
                type: 'range',
                range: () => ({ min: 1, max: 20, step: 1 }),
            },
            maxTimeRange: {
                name: 'Max Time Range [s]',
                type: 'number',
                range: () => ({ min: 1, step: 1 }),
            },
            series: ref([
                {
                    topicName: {
                        name: 'Topic Name',
                        type: 'select',
                        possibleValues: function () {
                            if (!this.rosStore) {
                                this.rosStore = useRosStore()
                                this.rosStore.searchTopics()
                            }
                            return this.rosStore.topicsList
                        },
                        rosStore: undefined,
                    },
                    messageProperty: {
                        name: 'Message Property',
                        type: 'text',
                    },
                    label: {
                        name: 'Label',
                        type: 'text',
                        hide: ref(true),
                    },
                },
            ]),
            update: function (
                name,
                value,
                arrayName = undefined,
                index = undefined
            ) {
                // Array update (not element)
                if (arrayName !== undefined && index === undefined) {
                    if (name === 'push' || name === 'pop') {
                        value = parseInt(value) + this.series.value.length
                        if (value < 1 || value > 6) return

                        // Hiding unused label
                        this.series.value[0].label.hide = value <= 1

                        while (value != this.series.value.length) {
                            if (this.series.value.length < value)
                                this.series.value.push({
                                    ...this.series.value[
                                        this.series.value.length - 1
                                    ],
                                })
                            else this.series.value.pop()
                        }
                    }
                }

                // Ignore array element update
                if (arrayName !== undefined || index !== undefined) return
            },
        },
        icon: 'mdi-chart-box-outline',
        // icon: 'mdi-chart-line',
        // icon: 'mdi-chart-box',
        // icon: 'mdi-chart-bell-curve-cumulative',
        // icon: 'mdi-chart-areaspline',
    },
}
