<template>
    <div class="battery">
        <h1>Battery Indicator Page</h1>
        <div id="container">
        <div id="indicator">
            <div id="percentage"></div>
        </div>
        <div style="background-color: black; height: 20vh; width:7%; border-radius: 0px 15px 15px 0px;"></div>
        </div>
        </div>
</template>

<script>
export default {
        name: "Battery",
        
        props: {
            'ws_address': String,
            'ros': Object,
        },

        data () {
          return {
          port: 8082,
          Ros: this.ros,
          topic: null,
          percentage: 0,
          };
        },

        computed: {

        },

        mounted() {
          // If ROS got disconnected, return to home
          if (this.ros==null) {
            this.$router.push({
              name: "Home",
            });
          }

          this.topic = new window.ROSLIB.Topic({
            ros: this.ros,
            name: '/battery_level',
            messageType: 'sensor_msgs/BatteryState'
          });
          this.topic.subscribe(function(message) {
                this.percentage = Math.round(message.percentage * 100)
                document.getElementById("percentage").innerHTML = this.percentage + " %"
                var rest = 100 - this.percentage
                if (this.percentage <= 25) {
                    document.getElementById("percentage").style.background = `linear-gradient(90deg, red ${this.percentage}%, grey 0%)`
                }
                else if (this.percentage <= 50) {
                    document.getElementById("percentage").style.background = `linear-gradient(90deg, orange ${this.percentage}%, grey 0%)`
                }
                else if (this.percentage <= 75) {
                    document.getElementById("percentage").style.background = `linear-gradient(90deg, yellow ${this.percentage}%, grey 0%)`
                }
                else {
                    document.getElementById("percentage").style.background = `linear-gradient(90deg, rgb(23, 187, 23) ${this.percentage}%, grey 0%)`
                }
                });
        },

        methods: {
        },
        beforeDestroy() {
          // Reloading page when going back
          window.location.reload();
        },
    };
</script>

<style scoped>
/* .battery {
    height: 100vh;
    padding: 15px;

    position: relative;
    display: flex;
    flex-direction: column;

    box-sizing: border-box;
    overflow: hidden;

    border-radius: 15px;
    background-color: blueviolet;
} */
#container {
    margin-left: auto;
    margin-right: auto;
    display: flex;
    align-items: center;
    height: 40vh;
    width: 40%;
    margin-top: 100px;
}
#indicator {
    margin-left: auto;
    margin-right: auto;
    display: flex;
    align-items: center;
    justify-content: center;
    height: 40vh;
    width: 100%;
    padding: 10px;
    border-radius: 15px;
    background-color: black;
    font-size: 60px;
    font-weight: 700;
}
#percentage {
    height: 100%;
    width: 100%;
    display: flex;
    align-items: center;
    justify-content: center;
    color: white;
    border-radius: 15px;
    background: linear-gradient(90deg, rgb(23, 187, 23));
}
</style>
