<template>
    <div>
        <div>
            <label>
                Keyboard Rover Steering
            </label>
        </div>
        <div>
            <div class="slidecontainer">
                <input type="range" min="1" max="100" value="50" class="slider" id="linear_speed" v-model="linear_speed_percentage">
                <label class="sliderLabel">Linear velocity: {{ this.linear_speed_percentage }}%</label>
            </div>
            <div class="slidecontainer">
                <input type="range" min="1" max="100" value="50" class="slider" id="angular_speed" v-model="angular_speed_percentage">
                <label class="sliderLabel">Angular velocity: {{ this.angular_speed_percentage }}%</label>
            </div>
        </div>
        <div class="keyboardControl">
            <p class="keyboardControl">
                <button class="keyboardControl" :class="{ pressed: pressed_W }">W</button>
            </p>
            <p class="keyboardControl">
                <button class="keyboardControl" :class="{ pressed: pressed_A }">A</button>
                <button class="keyboardControl" :class="{ pressed: pressed_S }">S</button>
                <button class="keyboardControl" :class="{ pressed: pressed_D }">D</button>
            </p>
        </div>
    </div>
</template>

<script>
    export default {
        name: "KeyboardControl",
        props: {
            'ws_address': String,
            'ros': Object,
        },
        data() {
            return {
                topic: null,
                max_linear_speed: 1,
                max_angular_speed: 1.57,
                linear_speed_percentage: 25,
                angular_speed_percentage: 25,
                timer: null,
                message_rate: 100, // [ms]
                pressed_W: false,
                pressed_A: false,
                pressed_S: false,
                pressed_D: false,
            }
        },
        methods: {
            startPublishing() {
                this.timer = setInterval(() => {
                    var message = new window.ROSLIB.Message({
                        linear : {
                        x : ( this.pressed_W - this.pressed_S ) * this.max_linear_speed * 0.01 * this.linear_speed_percentage,
                        y : 0,
                        z : 0
                        },
                        angular : {
                        x : 0,
                        y : 0,
                        z : ( this.pressed_A - this.pressed_D ) * this.max_angular_speed * 0.01 * this.angular_speed_percentage
                        }
                    });
                    this.topic.publish(message);
                }, this.message_rate)
            },
            keyListener(e, isPressed) {
                switch (e.key) {
                    case 'W':
                        this.pressed_W = isPressed;
                        break;
                    case 'S':
                        this.pressed_S = isPressed;
                        break;
                    case 'A':
                        this.pressed_A = isPressed;
                        break;
                    case 'D':
                        this.pressed_D = isPressed;
                        break;
                    default:
                        break;
                }
            }
        },
        mounted() {
            this.topic = new window.ROSLIB.Topic({
                ros : this.ros,
                name : '/cmd_vel',
                messageType : 'geometry_msgs/Twist'
            });
            window.addEventListener('keydown', this.keyListener($event, true));
            window.addEventListener('keyup', this.keyListener($event, false));
            startPublishing();
        },
        beforeDestroy() {
            clearInterval(this.timer);
            window.removeEventListener('keydown', this.keyListener($event, true));
            window.removeEventListener('keyup', this.keyListener($event, false));
        }
    };
</script>

<style>
    button.keyboardControl {
        margin: 5px;
        width: 50px;
        height: 50px;
        font-size: large;
        background-color: #ddd;
    }
    button.keyboardControl.pressed {
        background-color: #999;
    }
    div.keyboardControl {
        padding: 10px;
        text-align: center;
        background: #eee;
        margin: 0;
        display: inline-block;
        align-items: center;
    }
    p.keyboardControl {
        margin: 0;
    }
</style>
