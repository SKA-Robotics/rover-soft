<template>
    <div>
        <table width="100%" height="100%">
            <tr class="slidecontainer" height="60" v-for="i in this.elements_names.length">
                <td align="right" width="60%">
                    <input type="range" min="1" max="100" class="slider" :class="{ focused: focus_index == i - 1 }"
                    :id="elements_names[i - 1]" v-model="this.elements_effort_percentage[i - 1]" v-on:input="sliderInputCallback"
                    @click="focus_index = i - 1">
                </td>
                <td align="left" width="10%">
                    <label class="sliderLabel">{{ this.elements_text[i - 1] }}:</label>
                </td>
                <td align="left" width="20%">
                    <label class="sliderLabel">{{ this.elements_effort_percentage[i - 1] }}%</label>
                </td>
            </tr>
        </table>
        <div class="manipKeyboardControl">
            <p class="manipKeyboardControl">
                <button class="manipKeyboardControl" :class="{ pressed: pressed_Q }">Q</button>
                <button class="manipKeyboardControl" :class="{ pressed: pressed_W }">W</button>
                <button class="manipKeyboardControl" :class="{ pressed: pressed_E }">E</button>
            </p>
            <p class="manipKeyboardControl">
                <button class="manipKeyboardControl" :class="{ pressed: pressed_A }">A</button>
                <button class="manipKeyboardControl" :class="{ pressed: pressed_S }">S</button>
                <button class="manipKeyboardControl" :class="{ pressed: pressed_D }">D</button>
            </p>
        </div>
        <p>Use 'Space' to switch between sliders or 'Arrow Keys' to change each effort value.</p>
        <p>Steering will be automaticly switched between the arm or the claw by changing the choosen slider. Use keys:</p>
        <li>'A' and 'D' to rotation</li>
        <li>'W' and 'S' to lift</li>
        <li>'Q' and 'E' to tilt the arm or clamp the claw</li>
    </div>
</template>

<script>
import { capitalize } from '@vue/shared';
    export default {
        name: "ManipKeyboardControl",
        props: {
            'ws_address': String,
            'ros': Object,
        },
        data() {
            return {
                topic: null,
                timer: null,
                max_effort: 1.0,
                message_rate: 100, // [ms]
                pressed_Q: false,
                pressed_E: false,
                pressed_W: false,
                pressed_A: false,
                pressed_S: false,
                pressed_D: false,
                elements_effort_percentage: [25, 25, 25, 25, 25, 25],
                elements_text: ["Arm rotate effort", "Arm lift effort", "Arm tilt effort", "Claw rotate effort", "Claw lift effort", "Claw clamp effort"],
                elements_names: ["arm_rotate", "arm_lift", "arm_tilt", "claw_rotate", "claw_lift", "claw_clamp"],
                cookies_names: ["arm-rotate", "arm-lift", "arm-tilt", "claw-rotate", "claw-lift", "claw-clamp"],
                focus_index: 0
            }
        },
        methods: {
            startPublishing() {
                this.timer = setInterval(() => {
                    var current_time = new Date();
                    var message = new window.ROSLIB.Message({
                        header : {
                            stamp : {
                                secs : Math.floor(current_time.getTime()/1000),
                                nsecs : Math.round(1000000000 * (current_time.getTime()/1000 - Math.floor(current_time.getTime()/1000)))
                            }
                        },
                        name : this.elements,
                        effort : [ 0, 0, 0, 0, 0, 0 ]
                    });
                    if( this.focus_index >= 0 && this.focus_index <= 2 ){
                        message.effort[0] = ( this.pressed_A - this.pressed_D ) * this.max_effort * 0.01 * elements_effort_percentage[0];
                        message.effort[1] = ( this.pressed_W - this.pressed_S ) * this.max_effort * 0.01 * elements_effort_percentage[1];
                        message.effort[2] = ( this.pressed_Q - this.pressed_E ) * this.max_effort * 0.01 * elements_effort_percentage[2];
                    }
                    else if( this.focus_index >= 3 && this.focus_index <= 5 ){
                        message.effort[3] = ( this.pressed_A - this.pressed_D ) * this.max_effort * 0.01 * elements_effort_percentage[3];
                        message.effort[4] = ( this.pressed_W - this.pressed_S ) * this.max_effort * 0.01 * elements_effort_percentage[4];
                        message.effort[5] = ( this.pressed_Q - this.pressed_E ) * this.max_effort * 0.01 * elements_effort_percentage[5];
                    }
                    this.topic.publish(message);
                }, this.message_rate)
            },
            keyListener(e, isPressed) {
                var key = capitalize(e.key);
                switch (key) {
                    case 'Q':
                        this.pressed_Q = isPressed;
                        break;
                    case 'E':
                        this.pressed_E = isPressed;
                        break;
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
                            this.focus_index = (this.focus_index + 1) % this.elements_names.length;
                        document.getElementById(this.elements_names[this.focus_index]).focus();
                        break;
                    default:
                        break;
                }
            },
            sliderInputCallback() {
                this.$cookies.set('linear-speed-percentage', this.linear_speed_percentage);
                this.$cookies.set('angular-speed-percentage', this.angular_speed_percentage);
            }
        },
        mounted() {
            this.topic = new window.ROSLIB.Topic({
                ros : this.ros,
                name : '/manip_vel',
                messageType : 'sensor_msgs/JointState'
            });
            window.addEventListener('keydown', event => {
                this.keyListener(event, true);
            });
            window.addEventListener('keyup', event => {
                this.keyListener(event, false);
            });

            // Read previous percentage settings
            if (this.$cookies.isKey('linear-speed-percentage')) {
                this.linear_speed_percentage = this.$cookies.get('linear-speed-percentage');
            }
            if (this.$cookies.isKey('angular-speed-percentage')) {
                this.angular_speed_percentage = this.$cookies.get('angular-speed-percentage');
            }

            // Read maximum effort from ros params
            var base = "/web_interface/control"
            var maxEffortParam = new window.ROSLIB.Param({
                ros : this.ros,
                name :  base + '/max_effort' // do spradzenia czy istnieje ???
            });
            maxEffortParam.get((value) => {
                if (value != null) {
                    this.max_effort = value;
                }
            });
            this.startPublishing();

            // Set focus on first slider
            document.getElementById(this.elements_names[this.focus_index]).focus();
        },
        beforeDestroy() {
            clearInterval(this.timer);
            window.removeEventListener('keydown');
            window.removeEventListener('keyup');
        }
    };
</script>

<style>
    button.manipKeyboardControl {
        margin: 8px;
        width: 80px;
        height: 80px;
        font-size: 40px;
    }
    button.manipKeyboardControl.pressed {
        background-color: var(--secondary-light);
    }
    div.manipKeyboardControl {
        padding: 10px;
        text-align: center;
        background: #eee;
        margin: 50px;
        display: inline-block;
        align-items: center;
        border-radius: 15px;
    }
    p.manipKeyboardControl {
        margin: 0;
    }
    .slider.focused {
        background: var(--secondary-light);
        height: 12px;
    }
</style>
