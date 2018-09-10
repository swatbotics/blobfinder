#!/usr/bin/env python

# Boilerplate initialization stuff -- make sure to load the manifest
# from this package in order to be able to parse all of the correct
# messages. We will use math.pi to deal with angles, and random to
# implement direction changes for wandering.

import roslib; roslib.load_manifest('blobfinder')
import rospy

from blobfinder.msg import MultiBlobInfo

######################################################################
# Define a class to implement our node. This gets instantiated once
# when the node is run (see very bottom of file)

class blobtest:

    ##################################################################
    # Constructor for our class. Here we initialize any variables
    # which need to be accessed across multiple functions - note that
    # each such member variable is referred to with "self."

    def __init__(self, name):
        # Subscribe to blob messages to learn about blobs
        rospy.Subscriber('/blobfinder/blue_tape/blobs', 
                         MultiBlobInfo, self.blob_callback)

    #################################################################
    # Called when joystick messages come in

    def blob_callback(self, data):

        num = len(data.blobs)
        rospy.loginfo('got a message with %d blobs', num)
        for i in range(num):
            rospy.loginfo('  blob with area %f at (%f, %f)', 
                          data.blobs[i].area,
                          data.blobs[i].cx,
                          data.blobs[i].cy)

######################################################################
# Main program simply creates a class instance and then calls
# rospy.spin().

if __name__ == '__main__':

    rospy.init_node('blobtest')
    blobtest(rospy.get_name())

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
