#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import tf
from cv_bridge import CvBridge
from fastsense_msgs.msg import object_msg, img_points_msg
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
from object_detector_tools import get_object_points
from object_detector_tools import quaternion_to_list
from object_detector_tools import highlight_object
from object_detector_tools import swap_img_points


class ObjectDetector():
    """Base class for all detectors"""

    def __init__(self, detector_name):
        rospy.init_node(detector_name, anonymous=True)

        camera_info_topic = rospy.get_param('~camera_info_topic_name', '/camera/rgb/camera_info')
        img_out_topic = rospy.get_param('~img_topic_out', '/object_detector/front')
        self.image_topic = rospy.get_param('~camera_image_topic', '/camera/rgb/image_raw')
        self.camera_type = rospy.get_param('~camera_type', 'front')
        self.detect = rospy.get_param('~detect_state', True)

        self.camera_sub = rospy.Subscriber(self.image_topic, Image, self.camera_image_callback)
        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo,
                                                self.camera_info_callback)
        self.detect_state_sub = rospy.Subscriber(rospy.get_name() + '/detect', Bool,
                                                 self.detect_state_callback)

        self.object_msg_pub = rospy.Publisher(detector_name + "/info", object_msg, queue_size=10)
        self.img_points_pub = rospy.Publisher(rospy.get_name() + "/img_points", img_points_msg,
                                              queue_size=10)
        self.image_publisher = rospy.Publisher(img_out_topic, Image, queue_size=10)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.bridge = CvBridge()
        self.camera_matrix = None  # camera matrix or (camera) projection matrix
        self.camera_distortion = None  # camera distortion coefficients
        self.camera_frame = None
        self.image = None  # the last image that came from the camera (current)
        self.image_out = None  # the image in which the object is highlighted
        self.stamp = None  # time stamp (from image) for msg

    def camera_image_callback(self, frame):
        """Callback for camera images, must be override in every child class"""
        raise NotImplementedError()

    def camera_info_callback(self, info):
        """ Getting camera matrix and distortion coefficients """

        self.camera_matrix = np.ndarray(shape=(3, 3), dtype=float, buffer=np.array(info.K))
        self.camera_distortion = np.array(info.D[0:])
        self.camera_info_sub.unregister()

    def detect_state_callback(self, msg):
        """Updates the detect field"""
        self.detect = msg.data

    def process_object(self, img_points, tf_name, type_, id_='', object_size=0.3):
        """Calling all functions to process object"""

        object_points = get_object_points(object_size)
        object_pose = self.get_object_pose(object_points, img_points)
        self.send_transform(object_pose, tf_name)
        self.send_object_info(object_pose, type_=type_, id_=id_)  #
        self.image_out = highlight_object(self.image_out, img_points, type_=type_, data=id_)

    def get_object_pose(self, object_points, image_points):
        """Calculate object pose relative to camera frame"""

        object_pose = Pose()
        # convert dtype to float64 for solvePnP
        object_points = np.array(object_points, dtype="float64")
        image_points = np.array(image_points, dtype="float64")

        rot_vec, trans_vec = cv2.solvePnP(object_points, image_points,
                                          self.camera_matrix, self.camera_distortion)[1:]

        object_pose.position.x, object_pose.position.y, object_pose.position.z = trans_vec
        q = quaternion_from_euler(rot_vec[0], rot_vec[1], rot_vec[2])
        object_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        return object_pose

    def send_transform(self, object_pose, tf_name):
        """Send tf transform """

        orientation = quaternion_to_list(object_pose.orientation)
        tf_name = self.camera_type + "_" + tf_name
        tvec = (object_pose.position.x, object_pose.position.y, object_pose.position.z)
        self.tf_broadcaster.sendTransform(tvec, orientation,
                                          rospy.Time.now(),
                                          tf_name,
                                          self.camera_frame)

    def send_object_info(self, object_pose, type_, id_=''):
        """Sends a message about the position of the object, its type, optional ID"""

        msg = object_msg()
        msg.header.frame_id = self.camera_frame
        msg.header.stamp = self.stamp
        msg.pose_stamped.header.frame_id = self.camera_frame
        msg.pose_stamped.header.stamp = self.stamp  # rospy.Time.now()
        msg.type = type_
        msg.pose_stamped.pose = object_pose
        msg.camera_type = self.camera_type
        msg.camera_frame = self.camera_frame
        if id_ != '':
            msg.id = int(id_)
        self.object_msg_pub.publish(msg)

    def remove_distortion(self, image):
        """The function which removes distortion from image"""

        h, w = image.shape[:2]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(self.camera_matrix,
                                                         self.camera_distortion[1:],
                                                         np.eye(3),
                                                         self.camera_matrix, (w, h),
                                                         cv2.CV_16SC2)
        image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT)
        return image

    def publish_img_points(self, ids_, points_):
        msg = img_points_msg()
        msg.header.frame_id = self.camera_frame
        msg.header.stamp = self.stamp
        msg.IDs = ids_
        msg.points.data = swap_img_points(points_)
        self.img_points_pub.publish(msg)
