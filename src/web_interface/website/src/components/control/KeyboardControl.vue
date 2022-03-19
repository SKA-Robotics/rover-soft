<template>
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
        <p>
            <button class="keyboardControl" @press="goForward">W</button>
        </p>
        <p>
            <button class="keyboardControl" @press="turnLeft">A</button>
            <button class="keyboardControl" @press="goBackward">S</button>
            <button class="keyboardControl" @press="turnRight">D</button>
        </p>
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
            }
        },
        methods: {
            goForward() {

            },
            goBackward() {

            },
            turnLeft() {

            },
            turnRight() {

            }
        },
        mounted() {
            this.topic = new window.ROSLIB.Topic({
                ros : this.ros,
                name : '/cmd_vel',
                messageType : 'geometry_msgs/Twist'
            });
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