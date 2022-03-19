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
                <button class="keyboardControl" @press="goForward">W</button>
            </p>
            <p class="keyboardControl">
                <button class="keyboardControl" @press="turnLeft">A</button>
                <button class="keyboardControl" @press="goBackward">S</button>
                <button class="keyboardControl" @press="turnRight">D</button>
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
                pressed_W: false,
                pressed_A: false,
                pressed_S: false,
                pressed_D: false,
            }
        },
        methods: {
            goForward() {this.pressed_W = true;},
            goBackward() {this.pressed_S = true;},
            turnLeft() {this.pressed_A = true;},
            turnRight() {this.pressed_D = true;},
            stopForward() {this.pressed_W = false;},
            stopBackward() {this.pressed_S = false;},
            stopLeft() {this.pressed_A = false;},
            stopRight() {this.pressed_D = false;},
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
                    this.stopForward();
                    this.stopBackward();
                    this.stopLeft();
                    this.stopRight();
                }, 20) // message rate: 20 ms
            },
            keyListener(e) {
                switch (e.key) {
                    case 'w':
                        this.goForward();
                        break;
                    case 's':
                        this.goBack();
                        break;
                    case 'a':
                        this.turnLeft();
                        break;
                    case 'd':
                        this.turnRight();
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
            window.addEventListener('keypress', this.keyListener);
        },
        beforeDestroy() {
            clearInterval(this.timer);
            window.removeEventListener('keypress', this.keyListener);
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
