#!/usr/bin/env python

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import diagnostic_updater
import diagnostic_msgs
from std_msgs.msg import Float32

def dummy_diagnostic(stat):
    stat.add("Status", "nothing to be wrong")
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
    return stat

if __name__=='__main__':
    rospy.init_node("dummy_diagnostics")
    updater = diagnostic_updater.Updater()
    updater.setHardwareID("Device-%i-%i" % (27, 46) )
    updater.add("Dummy", dummy_diagnostic)
    updater.force_update()
    while not rospy.is_shutdown():
        rospy.sleep(0.5)
        updater.update()