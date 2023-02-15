import TestWindow from './TestWindow.vue'
import CameraWindow from './CameraWindow.vue'
import ChartWindow from './ChartWindow.vue'
import { useGstreamerStore, useRosStore } from '@/stores'
import { Service, ServiceRequest } from 'roslib'
import { ref } from 'vue'

const topicsList = ref([])

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
            topicName: {
                name: 'Topic Name',
                type: 'select',
                possibleValues: () => {
                    setTimeout(() => {
                        const rosStore = useRosStore()
                        const topicsClient = new Service({
                            ros: rosStore.ws,
                            name: '/rosapi/topics',
                            serviceType: 'rosapi/Topics',
                        })

                        const request = new ServiceRequest()

                        topicsClient.callService(request, (result) => {
                            // console.log(result)
                            topicsList.value = result.topics
                        })
                        // console.log('test')
                    }, 1000)
                    return topicsList.value
                },
            },
            messageProperty: {
                name: 'Message Property',
                type: 'text',
            },
            refreshingFrequency: {
                name: 'Refreshing Frequency',
                type: 'range',
                range: () => ({ min: 1, max: 20, step: 1 }),
            },
        },
    },
}
