<template>
    <div class="cameras">
        <TopicList
            :topics="sources"
            :layout="layout"
            @changeLayout="layout = $event"
        />
        <DragManager
            :host="wsUrl"
            :peers="peers"
            :layout="layout"
            @changeLayout="layout = $event"
        />
    </div>
</template>
<script>
import DragManager from '@/components/cameras/DragManager.vue'
import TopicList from '@/components/cameras/TopicList.vue'
export default {
    components: { DragManager, TopicList },
    name: 'Cameras',

    props: {
        ws_address: String,
    },

    data() {
        return {
            layout: {
                streams: [],
                variant: 0,
            },
            wsConn: null,
            peers: {},
            wsPort: '8443',
            wsServer: window.location.hostname,
        }
    },

    methods: {
        connect() {
            console.log('Connecting listener')
            this.wsConn = new WebSocket(this.wsUrl)
            this.wsConn.addEventListener('open', (event) => {
                this.wsConn.send(
                    JSON.stringify({
                        type: 'setPeerStatus',
                        roles: ['listener'],
                    })
                )
            })
            this.wsConn.addEventListener('error', this.serverError)
            this.wsConn.addEventListener('message', this.serverMessage)
            this.wsConn.addEventListener('close', this.serverClose)
        },
        serverMessage(event) {
            console.log('Received ' + event.data)
            const msg = JSON.parse(event.data)

            if (msg.type == 'welcome') {
                console.info(`Got welcomed with ID ${msg.peer_id}`)
                this.wsConn.send(
                    JSON.stringify({
                        type: 'list',
                    })
                )
            } else if (msg.type == 'list') {
                this.peers = {}
                msg.producers.forEach(({ id, meta }) => {
                    this.$set(this.peers, meta['display-name'], id)
                })
            } else if (msg.type == 'peerStatusChanged') {
                if (msg.roles.includes('producer')) {
                    this.$set(this.peers, msg.meta['display-name'], msg.peerId)
                } else if (msg.roles.length === 0) {
                    this.$delete(this.peers, msg.meta['display-name'])
                }
            }
            console.log(msg)
        },
        clearConnection() {
            this.wsConn.removeEventListener('error', this.serverError)
            this.wsConn.removeEventListener('message', this.serverMessage)
            this.wsConn.removeEventListener('close', this.serverClose)
            this.wsConn = null
        },
        serverClose(event) {
            this.clearConnection()
            this.peers = {}
            console.log('Close')
            window.setTimeout(this.connect, 1000)
        },
        serverError(event) {
            this.clearConnection()
            this.peers = {}
            console.log('Error', event)
            window.setTimeout(this.connect, 1000)
        },
    },

    computed: {
        webSocketHostname() {
            return this.ws_address
                ? new URL(this.ws_address).hostname
                : window.location.hostname
        },
        sources() {
            return Object.keys(this.peers)
        },
        wsUrl() {
            return `ws://${this.wsServer}:${this.wsPort}`
        },
    },

    mounted() {
        this.connect()
    },
    beforeDestroy() {
        this.clearConnection()
        // Reloading page when going back
        window.location.reload()
    },
}
</script>
<style scoped>
.cameras {
    height: 100vh;
    padding: 10px;

    position: relative;
    display: flex;
    flex-direction: column;

    box-sizing: border-box;
    overflow: hidden;
}
</style>
