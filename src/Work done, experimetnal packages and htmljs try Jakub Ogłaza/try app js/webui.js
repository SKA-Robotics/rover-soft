var twist;
var cmdVel;
var publishImmidiately = true;
var robot_IP;
var manager;
var teleop;
var ros;
function ruch(linear, angular) {
    if (linear !== undefined) {
        twist.linear.x = linear;
    } else {
        twist.linear.x = 0;
    }
    cmdVel.publish(twist);
}
function predkosc() {
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
        }
    });
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
    cmdVel.advertise();
}
function klawiatura() {
 
    if (teleop == null) {
        teleop = new KEYBOARDTELEOP.Teleop({
            ros: ros,
            topic: '/cmd_vel'
        });
    }
    robotSpeedRange = document.getElementById("robot-speed");
    robotSpeedRange.oninput = function () {
        teleop.scale = robotSpeedRange.value / 100
    }
}
window.onload = function () {
    robot_IP = "178.42.104.223";
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9090"
    });
predkosc();
klawiatura();
}