#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String
from arbotix_msgs.srv import SetSpeed


def set_config(dofs, param_name, type, value):

    service_names = ['/' + g + '/set_'+param_name for g in dofs]
    for sn in service_names:
        rospy.wait_for_service(sn)
    services = [rospy.ServiceProxy(sn, type) for sn in service_names]
    for s in services:
        s.call(value)


def goto_position(pose):

    if poses.has_key(pose):
        for dof in poses[pose].keys():
            pubs[dof].publish(poses[pose][dof])
        pass
    else:
        rospy.logerr("No such pose: %s " % pose)


def pose_request(msg_data):
    goto_position(msg_data.data)

if __name__ == "__main__":

    rospy.init_node("arm_mover")
    poses = rospy.get_param("~positions")
    joint_names = poses['rest'].keys()
    pose_values = poses['rest'].values()

    pubs = {}
    for d in joint_names:
        controller = '/'+d+'/command'
        pubs[d] = rospy.Publisher(controller, Float64)

    set_config(joint_names, 'speed', SetSpeed, 0.05)
    goto_position('rest')

    rospy.Subscriber("goto_position", String, pose_request)
    rospy.spin()
