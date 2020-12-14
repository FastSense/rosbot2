#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from object_detector.object_detector import ObjectDetector


class GateDetector(ObjectDetector):
    """Node for detecting and determining the pose of gates"""

    def __init__(self, node_name):
        ObjectDetector.__init__(self, node_name)
        self.gate_size_ = float(rospy.get_param('~gate_side_size', 1.5))

    def camera_image_callback(self, frame):
        """ """

        if not self.detect:
            return

        self.image = self.bridge.imgmsg_to_cv2(frame, "passthrough")
        self.image = cv2.bitwise_not(self.image)
        self.camera_frame = frame.header.frame_id
        self.stamp = frame.header.stamp
        self.image_out = self.image.copy()
        self.process_gate()
        image_message = self.bridge.cv2_to_imgmsg(self.image_out, encoding="passthrough")
        self.image_publisher.publish(image_message)

    def process_gate(self):
        """ """

        img_points = self.find_gate(self.image)
        if img_points is not None:
            tf_name = "gate"
            self.process_object(img_points, tf_name,
                                type_="gate", object_size=self.gate_size_)

            self.publish_img_points([i for i in range(0, len([img_points]))], [img_points])

    def find_gate(self, img):
        """Extracts contours and detects rectangles on them"""

        img = np.uint8(img)
        thresh = self.get_threshold(img)
        edges = cv2.Canny(img, thresh, thresh * 2)
        contours = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(approx)

            if len(approx) == 4:
                img_top_left = np.array([x, y])
                img_top_right = np.array([x + w, y])
                img_bottom_left = np.array([x, y + h])
                img_bottom_right = np.array([x + w, y + h])

                image_points = np.array([img_top_left, img_top_right,
                                         img_bottom_right, img_bottom_left])

                return image_points
        return None

    def get_threshold(self, img):
        """returns the threshold value obtained by the implementation of Otsu's algorithm"""

        image = cv2.GaussianBlur(img, (5, 5), 0)
        # Set total number of bins in the histogram
        bins_num = 256
        # Get the image histogram
        hist, bin_edges = np.histogram(image, bins=bins_num)
        # Calculate centers of bins
        bin_mids = (bin_edges[:-1] + bin_edges[1:]) / 2.
        # Iterate over all thresholds (indices) and get the probabilities w1(t), w2(t)
        weight1 = np.cumsum(hist)
        weight2 = np.cumsum(hist[::-1])[::-1]
        # Get the class means mu0(t)
        mean1 = np.cumsum(hist * bin_mids) / weight1
        # Get the class means mu1(t)
        mean2 = (np.cumsum((hist * bin_mids)[::-1]) / weight2[::-1])[::-1]
        inter_class_variance = weight1[:-1] * weight2[1:] * (mean1[:-1] - mean2[1:]) ** 2
        # Maximize the inter_class_variance function val
        index_of_max_val = np.argmax(inter_class_variance)
        threshold = bin_mids[:-1][index_of_max_val]
        # print("Otsu's algorithm implementation thresholding result: {}".format(threshold))
        return threshold


def main():
    gate_detector = GateDetector("gate_detector")
    rospy.loginfo("Gate detector started with parameters: \n"
                  "camera topic = {0}".format(gate_detector.image_topic))
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")


if __name__ == '__main__':
    main()
