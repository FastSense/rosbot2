#!/usr/bin/env python
import rospy
import numpy as np
from pyzbar import pyzbar
from object_detector.object_detector import ObjectDetector


class QRDetector(ObjectDetector):
    """Node for detecting and determining the pose of QR codes"""

    def __init__(self, node_name):
        ObjectDetector.__init__(self, node_name)
        self.QR_code_size = float(rospy.get_param('~QR_code_side_size', 0.3))

    def camera_image_callback(self, frame):
        """ """

        if not self.detect:
            return

        self.image = self.bridge.imgmsg_to_cv2(frame, "passthrough")
        self.camera_frame = frame.header.frame_id
        self.stamp = frame.header.stamp
        self.image_out = self.image.copy()
        self.process_qr_code()
        image_message = self.bridge.cv2_to_imgmsg(self.image_out, encoding="passthrough")
        self.image_publisher.publish(image_message)

    def process_qr_code(self):
        """Function for finding and processing QR codes"""

        image_points_dict = self.find_qr_code(self.image)
        for id_ in image_points_dict.keys():
            img_points = image_points_dict[id_]
            tf_name = "QR_" + id_
            self.process_object(img_points, tf_name, type_="QR",
                                id_=id_, object_size=self.QR_code_size)
        self.publish_img_points([int(i[0]) for i in image_points_dict.keys()],
                                image_points_dict.values())

    def find_qr_code(self, image):
        """Returns the angles of the all QR codes in image and its data"""

        barcodes = pyzbar.decode(image)
        image_points_dict = {}
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect  # extract the bounding box location of the barcode
            barcode_data = barcode.data.decode("utf-8")

            img_top_left = np.array([x, y])
            img_top_right = np.array([x + w, y])
            img_bottom_left = np.array([x, y + h])
            img_bottom_right = np.array([x + w, y + h])
            image_points = np.array([img_top_left, img_top_right,
                                     img_bottom_right, img_bottom_left])
            image_points_dict[barcode_data] = image_points

        return image_points_dict


def main():
    qr_detector = QRDetector("qr_detector")
    rospy.loginfo("QR detector started with parameters: \n"
                  "camera topic = {0}".format(qr_detector.image_topic))
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")


if __name__ == '__main__':
    main()
