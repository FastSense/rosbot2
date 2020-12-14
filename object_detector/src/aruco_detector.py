#!/usr/bin/env python
import rospy
import cv2
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from object_detector.object_detector import ObjectDetector
from object_detector.object_detector_tools import highlight_object


class ArucoDetector(ObjectDetector):
    """Node for detecting and determining the pose of aruco markers"""

    def __init__(self, node_name):
        ObjectDetector.__init__(self, node_name)

        self.camera_model_ = rospy.get_param('~camera_model', 'd435')
        self.marker_size = float(rospy.get_param('~marker_size', 1))
        self.parameters_ = cv2.aruco.DetectorParameters_create()
        self.aruco_dict_ = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    def camera_image_callback(self, frame):
        """"""

        if not self.detect:
            return

        self.image = self.bridge.imgmsg_to_cv2(frame, "passthrough")
        if self.camera_model_ == 'Fisheye' or self.camera_model_ == 't265':
            self.image = self.remove_distortion(self.image)

        self.camera_frame = frame.header.frame_id
        self.stamp = frame.header.stamp
        self.image_out = self.image.copy()
        self.find_aruco_markers(self.image)

        image_message = self.bridge.cv2_to_imgmsg(self.image_out, encoding="passthrough")
        self.image_publisher.publish(image_message)

    def find_aruco_markers(self, image):
        """function for finding aruco markers and processing them"""

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        markers_img_points, markers_ids, _ = cv2.aruco.detectMarkers(gray,
                                                                     self.aruco_dict_,
                                                                     parameters=self.parameters_)
        if markers_ids is None:
            return

        for i in range(0, markers_ids.size):
            tf_name = "aruco_" + str((markers_ids[i][0]))
            self.process_object(markers_img_points[i], tf_name, type_="aruco",
                                id_=int(markers_ids[i]), object_size=self.marker_size)

            self.publish_img_points([int(i[0]) for i in markers_ids], markers_img_points)

    def process_object(self, img_points, tf_name, type_, id_='', object_size=0.3):
        """Calling all functions to process object"""

        object_pose = self.get_object_pose(img_points)
        self.send_transform(object_pose, tf_name)
        self.send_object_info(object_pose, type_=type_, id_=id_)
        self.image_out = highlight_object(self.image_out,
                                          img_points[0],
                                          type_=type_,
                                          data=id_)

    def get_object_pose(self, img_points, image_points=''):
        """Calculate object pose relative to camera frame"""

        object_pose = Pose()
        rot_vec, trans_vec = cv2.aruco.estimatePoseSingleMarkers(img_points,
                                                                 self.marker_size,
                                                                 self.camera_matrix,
                                                                 self.camera_distortion)

        object_pose.position.x, object_pose.position.y, object_pose.position.z = trans_vec[0][0]
        q = quaternion_from_euler(rot_vec[0][0][0], rot_vec[0][0][1], rot_vec[0][0][2])
        object_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        return object_pose


def main():
    aruco_detector = ArucoDetector("aruco_detector")
    rospy.loginfo("Aruco detector started with parameters: \n"
                  "camera topic = {0}".format(aruco_detector.image_topic))
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")


if __name__ == '__main__':
    main()
