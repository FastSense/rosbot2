#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from find_object_2d.msg import ObjectsStamped
from object_detector.object_detector import ObjectDetector


class AerobotLogoDetector(ObjectDetector):
    """Node for detecting and determining the pose of aerbot logo"""

    def __init__(self, node_name):
        ObjectDetector.__init__(self, node_name)
        img_topic_name = rospy.get_param('~img_topic_name',
                                         '/object_detector/rgb/image_raw')
        self.logo_size = float(rospy.get_param("~logo_side_size", 0.3))

        self.find_object2d_sub = rospy.Subscriber('/objectsStamped', ObjectsStamped,
                                                  self.find_object_2d_callback)
        self.img_to_find_object2d_pub = rospy.Publisher(img_topic_name, Image, queue_size=10)

    def camera_image_callback(self, frame):
        """ """

        if not self.detect:
            return

        self.image = self.bridge.imgmsg_to_cv2(frame, "passthrough")
        self.camera_frame = frame.header.frame_id
        self.stamp = frame.header.stamp
        self.image_out = self.image.copy()
        self.img_to_find_object2d_pub.publish(frame)

    def find_object_2d_callback(self, msg):
        """callback for find_object_2d"""

        data = msg.objects.data
        array_step = 12  # step between elements in std_msgs/MultiArrayLayout msg
        tf_name = "aerobot_logo"
        all_logos_points = np.array([])
        # iterate through all found objects from find_object_2d
        for i in range(0, len(data), array_step):
            top_left, top_right, bottom_left, bottom_right = self.get_corners_from_data(data, i)

            img_top_left = np.array([top_left[0], top_left[1]])
            img_top_right = np.array([top_right[0], top_right[1]])
            img_bottom_left = np.array([bottom_left[0], bottom_left[1]])
            img_bottom_right = np.array([bottom_right[0], bottom_right[1]])

            image_points = np.array([img_top_left, img_top_right,
                                     img_bottom_right, img_bottom_left])
            np.append(all_logos_points, image_points)
            self.process_object(image_points, tf_name, type_="aerobot_logo",
                                object_size=self.logo_size)

        image_message = self.bridge.cv2_to_imgmsg(self.image_out, encoding="passthrough")
        self.image_publisher.publish(image_message)

        if len(all_logos_points) > 0 and len(image_points) > 0:
            self.publish_img_points([int(i) for i in range(len(all_logos_points))], image_points)

    def get_corners_from_data(self, data, index):
        """ Return image_points from ObjectsStamped msg unit """

        i = index
        object_height = data[i + 1]
        object_width = data[i + 2]
        homography_matrix = np.ndarray(shape=(3, 3), dtype=float,
                                       buffer=np.array(data[i + 3:i + 12]))

        img_top_left = self.get_img_corner_from_homography_matrix(homography_matrix, 0, 0)
        img_top_right = self.get_img_corner_from_homography_matrix(homography_matrix,
                                                                   object_width, 0)
        img_bottom_left = self.get_img_corner_from_homography_matrix(homography_matrix,
                                                                     0, object_height)
        img_bottom_right = self.get_img_corner_from_homography_matrix(homography_matrix,
                                                                      object_width, object_height)

        return img_top_left, img_top_right, img_bottom_left, img_bottom_right

    def get_img_corner_from_homography_matrix(self, h_matrix, point_x, point_y):
        """Transform and return corner point (x,y) from homography matrix"""

        corner_x = h_matrix[0][0] * point_x + h_matrix[1][0] * point_y + h_matrix[2][0]
        corner_y = h_matrix[1][1] * point_y + h_matrix[0][1] * point_x + h_matrix[2][1]
        norm = h_matrix[0][2] * point_x + h_matrix[1][2] * point_y + h_matrix[2][2]
        corner_x = corner_x / norm
        corner_y = corner_y / norm

        return [corner_x, corner_y]


def main():
    aerobot_logo_detector = AerobotLogoDetector("aerobot_logo_detector")
    rospy.loginfo("Aerobot logo detector started with parameters: \n"
                  "camera topic = {0}".format(aerobot_logo_detector.image_topic))
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")


if __name__ == '__main__':
    main()
