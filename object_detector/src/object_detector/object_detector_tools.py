import tf
import cv2
import rospy
import numpy as np


def transform_pose(object_msg, tf_buffer, dst_frame="local_origin", time_duration=0.1):
    """Transform pose from camera_frame to local_origin frame"""

    try:
        tf_buffer.waitForTransform(object_msg.camera_frame, dst_frame,
                                   rospy.Time(0), rospy.Duration(time_duration))
        object_msg.pose_stamped = tf_buffer.transformPose(dst_frame,
                                                          object_msg.pose_stamped)
        return object_msg.pose_stamped
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return


def get_pose_from_transform(object_msg, tf_buffer, dst_frame="local_origin",
                            time_duration=0.2):
    """ Get fobject pose from tf_transform """

    if object_msg.id != 0:
        tf_name = (object_msg.camera_type + "_" + object_msg.type + "_" + str(object_msg.id))
    else:
        tf_name = object_msg.camera_type + "_" + object_msg.type

    try:
        trans_vec = tf_buffer.lookup_transform(dst_frame, tf_name, rospy.Time(),
                                               rospy.Duration(time_duration)).transform.translation

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return tf.LookupException

    object_msg.pose_stamped.pose.position.x = trans_vec.x
    object_msg.pose_stamped.pose.position.y = trans_vec.y
    object_msg.pose_stamped.pose.position.z = trans_vec.z

    return object_msg.pose_stamped


def get_object_points(side_size):
    """ Returns the object points corresponding to its dimensions """

    obj_top_left = [-side_size / 2, side_size / 2, 0]
    obj_top_right = [side_size / 2, side_size / 2, 0]
    obj_bottom_left = [-side_size / 2, -side_size / 2, 0]
    obj_bottom_right = [side_size / 2, -side_size / 2, 0]
    object_points = np.array([obj_top_left, obj_top_right,
                              obj_bottom_right, obj_bottom_left])
    return object_points


def quaternion_to_list(quaternion):
    """ Converting quaternion to list
        (cuz we can't iterate through quaternion) """

    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def highlight_object(image, points, type_="", data=""):
    """color highlighting of object outlines"""

    points = np.array(points, dtype="int64")
    cv2.rectangle(image, tuple(points[0]), tuple(points[-2]), (0, 0, 255), 2)
    text = "{} {}".format(type_, data)
    cv2.putText(image, text, (points[0][0], points[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 255), 2)
    return image


def swap_img_points(img_points):
    """Swaps the elements in the array in the required order
     so that the array is suitable for the publish_img_points function"""

    result = list()
    for item in img_points:
        corners = list()
        x, y, w, h = cv2.boundingRect(item)
        corners.append([int(x), int(y)])  # top left
        corners.append([int(x + w), int(y)])  # top right
        corners.append([int(x), int(y + h)])  # bottom left
        corners.append([int(x + w), int(y + h)])  # bottom right

        for point in corners:
            result.append(point[0])  # x
            result.append(point[1])  # y

    return result
