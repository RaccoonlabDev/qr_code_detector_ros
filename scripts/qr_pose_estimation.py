#!/usr/bin/env python
# coding=utf8

import os.path
import cv2

import rospy
from cv_bridge import CvBridge, CvBridgeError
import math

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros
import tf

import qr_code_processing
from qr_code_detector_ros.msg import QrCode
import numpy as np

bridge = CvBridge()
cv_image = Image()

topic_tf_child = "qr_code"
topic_tf_perent = "base_link" #get from image msgs
map_id = "map"

t = TransformStamped()
tf2_br = tf2_ros.TransformBroadcaster()

timer = 0.
qr_proc = None
blur_threshold = 30        # the higher the value, the more sensitive the filter

def image_clb(data):
    """
    get image from ros
    :type data: Image
    :return:
    """
    global bridge, cv_image, get_image_flag, topic_tf_perent, _rate, timer, old_time

    timer += (rospy.Time.now() - old_time).to_sec()
    old_time = rospy.Time.now()
    if _rate != 0 and timer < 1./_rate:
        return


    topic_tf_perent = data.header.frame_id

    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        if is_blur(cv_image):
            "print is blur"
            return
        timer = 0.
        get_image_flag = True
    except CvBridgeError as e:
        print(e)

def q_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return np.array([x, y, z, w])

def pubTf(position, orientation, data):
    """
    publish find object to tf2
    :param position:
    :param orientation:
    :return:
    """
    global t
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = topic_tf_perent
    t.child_frame_id = topic_tf_child+str(data)
    t.transform.translation.x = position[0]
    t.transform.translation.y = position[1]
    t.transform.translation.z = position[2]

    q = tf.transformations.quaternion_from_euler(orientation[0],orientation[1],orientation[2])

    if isVertical:
        rotation_quaternion = tf.transformations.quaternion_from_euler(math.pi, math.pi * 0.5, 0.)
        quaternion = q_mult(q, rotation_quaternion)

    else:
        rotation_quaternion = tf.transformations.quaternion_from_euler(math.pi, 0., (math.pi * 0.5))
        quaternion = q_mult(q, rotation_quaternion)
        # quaternion = tf.transformations.quaternion_from_euler(orientation[0]+math.pi, orientation[1], orientation[2]+(math.pi*0.5))

    t.transform.rotation.x = quaternion[0]
    t.transform.rotation.y = quaternion[1]
    t.transform.rotation.z = quaternion[2]
    t.transform.rotation.w = quaternion[3]

    tf2_br.sendTransform(t)

def sub_TF(data):
    """
    get position "map_id" --> "topic_tf_child" is availibale
    :param map_id:
    :return:
    """
    try:
        trans = tfBuffer.lookup_transform(map_id,topic_tf_child+str(data), rospy.Time())
        # print "tf OK:", trans
        qr_code_msgs = QrCode()
        qr_code_msgs.data = data

        pose_sp_msgs = PoseStamped()
        pose_sp_msgs.header.frame_id = map_id
        pose_sp_msgs.header.stamp = rospy.Time.now()
        pose_sp_msgs.pose.position.x = trans.transform.translation.x
        pose_sp_msgs.pose.position.y = trans.transform.translation.y
        pose_sp_msgs.pose.position.z = trans.transform.translation.z

        pose_sp_msgs.pose.orientation.x = trans.transform.rotation.x
        pose_sp_msgs.pose.orientation.y = trans.transform.rotation.y
        pose_sp_msgs.pose.orientation.z = trans.transform.rotation.z
        pose_sp_msgs.pose.orientation.w = trans.transform.rotation.w

        qr_code_msgs.pose = pose_sp_msgs
        qr_code_pub.publish(qr_code_msgs)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print "TF listener error"
            return

def camera_info_clb(data):
    """
    Get cam info
    :param data:
    :return:
    """
    global qr_proc, camera_info_sub
    if qr_proc is None:
        return
    t = 0
    for i in range(3):
        for k in range(3):
            qr_proc.camera_parameters[i][k] = data.K[t]
            t+=1
    for i in range(5):
        qr_proc.camera_distortion_param[i] = data.D[i]
    camera_info_sub.unregister()
    print("get camera params")

def is_blur(image):
    """
	Retund image state
	:param image:
	:param threshold:
	:return:
	"""
    global blur_threshold
    val = cv2.Laplacian(image, cv2.CV_64F).var()
    print "blur", val, blur_threshold
    if val < blur_threshold:
        return True
    else:
        return False

############
### Main ###
############
if __name__ == '__main__':
    rospy.init_node('image_pose_estimation_node', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    _rate = 10.                 # this is a voracious application, so I recommend to lower the frequency, if it is not critical

    max_dist = 10.              # publish objects that are no further than the specified value

    size_image = 0.252             # the width of the image in meters

    show_image = True           # show image in window
    camera_name = "down_camera"      # the name of the camera in ROS
    isVertical = False
    # init params
    camera_name = rospy.get_param("~camera_name", camera_name)
    topic_tf_child = rospy.get_param("~frame_id", topic_tf_child)
    show_image = rospy.get_param("~show_image", show_image)

    size_image = rospy.get_param("~size_image", size_image)
    map_id = rospy.get_param("~map_id", map_id)

    blur_threshold = rospy.get_param("~blur_threshold", blur_threshold)
    max_dist = rospy.get_param("~max_dist", max_dist)
    isVertical = rospy.get_param("~is_vertical", isVertical)

    _rate = rospy.get_param("~rate", _rate)
    _rate = -1. if _rate <= 0. else _rate


    # init params
    get_image_flag = False

    qr_proc = qr_code_processing.QrCodeEstimation(blur_threshold, size_image, show_image)
    qr_proc.max_dist = max_dist

    old_time = rospy.Time.now()

    # Subscribers
    rospy.Subscriber(camera_name+"/image_raw", Image, image_clb)
    camera_info_sub = rospy.Subscriber(camera_name+"/camera_info", CameraInfo, camera_info_clb)
    # Publisher
    qr_code_pub = rospy.Publisher("find_qr", QrCode, queue_size=10)
    image_pub = rospy.Publisher("qr_code/image", Image)

    while not rospy.is_shutdown():

        if get_image_flag:
            frame, trans, rot, data = qr_proc.update(cv_image)

            if trans is not None and rot is not None:
                pubTf(trans,rot, data)
            # Subscribe ad publish tf
            sub_TF(data)
            if show_image:
                try:
                    image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
                except CvBridgeError as e:
                    print(e)

            get_image_flag = False
