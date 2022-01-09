<template>
    <div class="control">
      <body style="background-color:lightgrey;">
        <h1>This is the control page.</h1>
        <p>This the place where should work control programme and gui app</p>
        <p>To move your rover use wasd or choose one of the option below</p>
        <p>Your programme uses KeyboardControl vue in which you should have java script source code</p>
<span>Selected: {{ selected }}</span>
       <keyboard-control
          :ws_address="this.ws_address"
          :ros="this.ros"/>
    </div>
</template>
<script>
    import KeyboardControl from '@/KeyboardControl.vue';
    export default {
        name: "Control",
        
        components: {
          KeyboardControl,
        },
        props: {
            'ws_address': String,
            'ros': Object,
        },

        mounted() {
          // If ROS got disconnected, return to home
          if (this.ros==null) {
            this.$router.push({
              name: "Home",
            });
          }
        },

        beforeDestroy() {

          // Reloading page when going back
          window.location.reload();
        },
    };
</script>
