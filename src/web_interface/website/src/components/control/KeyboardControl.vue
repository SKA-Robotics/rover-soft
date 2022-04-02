<template>
    <div>
        <div>
            <h2>
                Keyboard Rover Steering
            </h2>
        </div>
        <div>
            <div class="slidecontainer">
                <input type="range" min="1" max="100" class="slider" id="linear_speed" v-model="linear_speed_percentage">
                <label class="sliderLabel">Linear velocity: {{ this.linear_speed_percentage }}%</label>
            </div>
            <div class="slidecontainer">
                <input type="range" min="1" max="100" class="slider" id="angular_speed" v-model="angular_speed_percentage">
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
        <p>Use 'Space' to switch between sliders or 'Arrow Keys' to change each speed value.</p> 
    </div>
</template>

<script>
import { capitalize } from '@vue/shared';
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
                elements: ["linear_speed", "angular_speed"],
                focus_index: 0
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
                var key = capitalize(e.key);
                switch (key) {
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
                    case ' ':
                        if(isPressed)
                            this.focus_index = (this.focus_index + 1) % this.elements.length;
                        document.getElementById(this.elements[this.focus_index]).focus();
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
            window.addEventListener('keydown', event => {
                this.keyListener(event, true);
            });
            window.addEventListener('keyup', event => {
                this.keyListener(event, false);
            });
            this.startPublishing();
            document.getElementById(this.elements[this.focus_index]).focus();
        },
        beforeDestroy() {
            clearInterval(this.timer);
            window.removeEventListener('keydown');
            window.removeEventListener('keyup');
        }
    };
</script>

<style>
    button.keyboardControl {
        margin: 8px;
        width: 80px;
        height: 80px;
        font-size: 40px;
    }
    button.keyboardControl.pressed {
        background-color: var(--secondary-light);
    }
    div.keyboardControl {
        padding: 10px;
        text-align: center;
        background: #eee;
        margin: 50px;
        display: inline-block;
        align-items: center;
    }
    p.keyboardControl {
        margin: 0;
    }
</style>
