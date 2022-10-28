<template>
    <div
        :style="{
            left: stream.x + 'px',
            top: stream.y + 'px',
            width: stream.width + 'px',
            height: stream.height + 'px',
            'background-color': 'black',
            position: 'absolute',
            display: 'flex',
            'align-items': 'center',
            'justify-content': 'center',
        }"
    >
        <video
            preload="none"
            ref="viewer"
            :style="{
                width: dimensions.width + 'px',
                height: dimensions.height + 'px',
                'object-fit': 'cover',
            }"
        />
    </div>
</template>
<script>
export default {
    props: {
        host: String, // should be a global
        stream: Object,
        peer: String,
    },
    data() {
        return {
            wsConn: null,
            peerConnection: null,
            dataChannel: null,
            videoDim: {
                width: null,
                height: null,
            },
        }
    },
    methods: {
        connect() {
            console.log('Connecting to server ' + this.host)
            this.wsConn = new WebSocket(this.host)
            this.wsConn.addEventListener('open', this.connectPeer)
            this.wsConn.addEventListener('error', this.serverError)
            this.wsConn.addEventListener('message', this.serverMessage)
            this.wsConn.addEventListener('close', this.serverClose)
        },
        connectPeer() {
            console.log('Connecting ' + this.peer_id)

            this.wsConn.send(
                JSON.stringify({
                    type: 'startSession',
                    peerId: this.peer,
                })
            )
        },
        createCall(msg) {
            console.log('Creating RTCPeerConnection')

            this.peerConnection = new RTCPeerConnection({
                iceServers: [
                ],
            })
            this.peerConnection.ontrack = this.remoteStreamAdded

            console.log('Created peer connection for call, waiting for SDP')
        },
        incomingICE(ice) {
            this.peerConnection.addIceCandidate(new RTCIceCandidate(ice))
        },
        incomingSDP(sdp) {
            this.peerConnection
                .setRemoteDescription(sdp)
                .then(this.remoteDescriptionSet)
        },
        remoteDescriptionSet() {
            console.log('Got SDP offer')
            this.peerConnection.createAnswer().then(this.localDescription)
        },
        localDescription(desc) {
            console.log('Got local description: ' + JSON.stringify(desc))
            this.peerConnection.setLocalDescription(desc).then(() => {
                console.log('Sending SDP answer')
                this.wsConn.send(
                    JSON.stringify({
                        type: 'peer',
                        sessionId: this.id,
                        sdp: this.peerConnection.localDescription.toJSON(),
                    })
                )
            })
        },
        remoteStreamAdded(event) {
            console.log(event)
            let videoTracks = event.streams[0].getVideoTracks()
            let audioTracks = event.streams[0].getAudioTracks()

            console.log(videoTracks)

            if (videoTracks.length > 0) {
                console.log(
                    'Incoming stream: ' +
                        videoTracks.length +
                        ' video tracks and ' +
                        audioTracks.length +
                        ' audio tracks'
                )
                this.$refs.viewer.srcObject = event.streams[0]
                this.$refs.viewer.play()
                setTimeout(() => {
                    console.error(this.$refs.viewer.videoHeight)
                    this.videoDim.width =
                        this.$refs.viewer && this.$refs.viewer.videoWidth
                    this.videoDim.height =
                        this.$refs.viewer && this.$refs.viewer.videoHeight
                }, 500)
            } else {
                console.log('Stream with unknown tracks added, resetting')
                this.endSession()
            }
        },
        serverMessage(event) {
            console.log('Received ' + event.data)
            const msg = JSON.parse(event.data)

            if (msg.type == 'registered') {
                console.log('Registered with server')
                this.connectPeer()
            } else if (msg.type == 'sessionStarted') {
                console.log('Session started')
                this.id = msg.sessionId
            } else if (msg.type == 'error') {
                this.endSession()
            } else if (msg.type == 'endSession') {
                this.endSession()
            } else if (msg.type == 'peer') {
                // Incoming peer message signals the beginning of a call
                if (!this.peerConnection) this.createCall(msg)

                if (msg.sdp != null) {
                    this.incomingSDP(msg.sdp)
                } else if (msg.ice != null) {
                    this.incomingICE(msg.ice)
                }
            }
        },
        serverClose(event) {
            console.log('Server closed', event)
            this.endSession()
        },
        serverError(event) {
            console.log('Server error', event)
            this.endSession()
        },
        endSession(retry = true) {
            this.wsConn.removeEventListener('open', this.connectPeer)
            this.wsConn.removeEventListener('error', this.serverError)
            this.wsConn.removeEventListener('message', this.serverMessage)
            this.wsConn.removeEventListener('close', this.serverClose)
            this.wsConn.close()
            this.wsConn = null
            this.peerConnection = null
            this.dataChannel = null

            if (retry) setTimeout(1000, () => this.connect())
        },
    },
    mounted() {
        this.connect()
    },
    beforeDestroy() {
        this.endSession(false)
    },
    computed: {
        dimensions() {
            console.log(this.videoDim)

            const aspectRatio = this.videoDim.width / this.videoDim.height
            let { width, height } = this.stream

            // room for the border
            width -= 10
            height -= 10

            // calculate dimensions for maxed width or height
            let maxWidth = {
                width,
                height: width / aspectRatio,
            }
            let maxHeight = {
                width: height * aspectRatio,
                height,
            }

            // choose the one that will fit
            if (this.stream.height < maxWidth.height) {
                return maxHeight
            } else {
                return maxWidth
            }
        },
    },
}
</script>
<style scoped></style>
