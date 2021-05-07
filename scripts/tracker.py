#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import numpy as np
from scipy.spatial import distance
import time
import rviz_util
import predictor
from timer import Timer
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion, PoseStamped, Vector3Stamped
from ros_3d_bb.msg import BoundingBox3D, BoundingBox3DArray
import tf2_ros
import tf2_geometry_msgs


VERBOSE = True
DEBUG = False
TIME = True


class BoundingBox:
    def __init__(self, x=0, y=0, z=0, size_x=0, size_y=0, size_z=0, bounding_box=None):
        """Create a bounding box with all values passed separately or using a BoundingBox3D message"""

        if not bounding_box:
            self.x = x
            self.y = y
            self.z = z
            self.size_x = size_x
            self.size_y = size_y
            self.size_z = size_z
        else:
            self.x = bounding_box.center.position.x
            self.y = bounding_box.center.position.y
            self.z = bounding_box.center.position.z
            self.size_x = bounding_box.size.x
            self.size_y = bounding_box.size.y
            self.size_z = bounding_box.size.z


class DetectedObject:
    def __init__(self, uid, bounding_box):
        """Creates a detected object based on the bounding box and UID"""

        # Set initial values in order to call self.update()
        # because velocity is calculated based on the previous self.x and self.y.
        self.x = bounding_box.x
        self.y = bounding_box.y

        self.update(bounding_box)
        self.uid = uid

        if DEBUG:
            rospy.loginfo("Created new object: " + str(self))

    def __eq__(self, other):
        return self.uid == other.uid

    def __hash__(self):
        return hash(self.uid)

    def __str__(self):
        return "UID: " + str(self.uid) + ", x: " + str(self.x) + ", y: " + str(self.y) + ", v_x: " + str(self.v_x) + ", v_y: " + str(self.v_y)

    def update(self, bounding_box):
        """Updates the object's position, scale, calculates the velocity, resets disappeared counter."""
        x, y = bounding_box.x, bounding_box.y
        self.v_x = x - self.x
        self.v_y = y - self.y
        self.x = x
        self.y = y
        self.z = bounding_box.z
        self.diameter = bounding_box.size_x
        self.height = bounding_box.size_y
        self.disappeared = 0

    def has_disappeared(self):
        self.disappeared += 1


class Tracker:
    def __init__(self, max_frames_disappeared=30, starting_id=0):
        """Initializes the tracker
        
        An optional disappearance threshold and starting ID for detections can be provided
        """

        self.max_frames_disappeared = max_frames_disappeared
        self.current_uid = starting_id

        self.objects = []

    def new_object(self, bounding_box):
        """Register a newly detected object."""
        self.objects.append(DetectedObject(self.current_uid, bounding_box))

        # Increment the UID for the next detected object.
        self.current_uid += 1

    def get_coordinates(self):
        """Returns coordinates of detected objects in a Python list"""
        # This can possibly be improved a bit with Numpy
        coordinates = []
        for detected_object in self.objects:
            if DEBUG:
                rospy.loginfo("Object: " + str(detected_object))
            coordinates.append((detected_object.x, detected_object.y))

        return coordinates

    def delete_if_disappeared(self, detected_object):
        """Deletes the long-disappeared objects (when time disappeared >= max_frames_disappeared)"""
        
        if detected_object.disappeared >= self.max_frames_disappeared:
            self.objects.remove(detected_object)

    def update(self, bounding_boxes: list):
        """Updates the positions of detected objects, add new objects and deletes long-disappeared objects."""
        
        # If there are no detections, all of the previous objects have disappeared.
        if len(bounding_boxes) == 0:
            for detected_object in self.objects:
                detected_object.disappeared()
                self.delete_if_disappeared(detected_object)
        # If there are no existing objects, just register all the detections.
        elif len(self.objects) == 0:
            for bounding_box in bounding_boxes:
                self.new_object(bounding_box)
        # Otherwise, find the closest matches between existing objects and detections.
        else:
            # Calculate the Euclidean distance between all the pairs of existing objects and detections.
            # Only coordinates needed for scipy.spatial.distance.cdist()
            current_coordinates = self.get_coordinates()

            # An example of the output of cdist():
            #           new_1   new_2
            # current_1   1       2
            # current_2   3       4
            distances = distance.cdist(
                np.array(current_coordinates), [(bb.x, bb.y) for bb in bounding_boxes], "euclidean")

            # The following part is mainly from the following article:
            # https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/
            # Slightly modified the given code, but it was already pretty optimal (using Numpy).

            # Basically, this finds indices with least distances between the frames
            # current == rows; new == columns;
            indices_sorted_current = distances.min(axis=1).argsort()
            indices_sorted_new = distances.argmin(
                axis=1)[indices_sorted_current]

            # Using set() to filter out the indices already used,
            # also enables set difference operation for later
            indices_used_current = set()
            indices_used_new = set()

            for i, j in zip(indices_sorted_current, indices_sorted_new):
                if i in indices_used_current or j in indices_used_new:
                    continue
                else:
                    self.objects[i].update(bounding_boxes[j])
                    indices_used_current.add(i)
                    indices_used_new.add(j)

            nr_of_current_objects = distances.shape[0]
            nr_of_new_objects = distances.shape[1]

            if nr_of_current_objects > nr_of_new_objects:
                # Finding the difference of sets == unused indices
                # Converting to list in order to use reversed() later
                indices_unused_current = list(set(
                    range(nr_of_current_objects)).difference(indices_used_current))

                # reversed() in order not to mess up the indexing
                # (iteration + mutation on the same list)
                for i in reversed(indices_unused_current):
                    obj = self.objects[i]
                    obj.has_disappeared()
                    self.delete_if_disappeared(obj)
            elif nr_of_current_objects < nr_of_new_objects:
                indices_unused_new = set(
                    range(nr_of_new_objects)).difference(indices_used_new)
                for j in indices_unused_new:
                    self.new_object(bounding_boxes[j])

        if VERBOSE:
            rospy.loginfo("Objects: " + str(list(map(str, self.objects))))

    def get_position_dict(self):
        position_dict = {}

        for detected_object in self.objects:
            position_dict[detected_object.uid] = (
                detected_object.x, detected_object.y)

        return position_dict

    def get_velocity_dict(self):
        velocity_dict = {}

        for detected_object in self.objects:
            velocity_dict[detected_object.uid] = (
                detected_object.v_x, detected_object.v_y)

        return velocity_dict


class RosTracker:
    def __init__(self):
        self.bounding_boxes = []
        self.max_frames_disappeared = 30
        self.time = time.time()
        self.framerate = 0

        # Initializing the tracker
        self.tracker = Tracker(self.max_frames_disappeared)

        # Initializing the predictor
        self.predictor = predictor.Predictor(self.tracker, sensitivity=0.2)

        # Defining the frame IDs
        self.frame_id_world = "world"
        self.frame_id_odom = "odom"
        self.frame_id_realsense = "realsense_mount"

        # Creating the transform broadcasters
        pose_world = Pose()
        pose_world.orientation.w = 1
        self.tf_publisher_world = rviz_util.TFPublisher(
            pose_world, "world", "odom")

        pose_realsense = Pose()
        pose_realsense.orientation = Quaternion(0.5, -0.5, 0.5, -0.5)
        pose_realsense.position.x = 0.17
        pose_realsense.position.z = 0.20
        self.tf_publisher = rviz_util.TFPublisher(
            pose_realsense, "base_link", "realsense_mount")
        
        # Creating the TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # For handling RViz visualization
        self.rviz = rviz_util.RViz(frame_id=self.frame_id_world)

        # Optionally, creating a list of timers (for performance measurement)
        if TIME:
            self.timers = []

        # Subscribed topics
        self.bb_point_topic = "/ros_3d_bb/bb_3d"

        # Subscribers
        self.bb_point_sub = rospy.Subscriber(
            self.bb_point_topic, BoundingBox3DArray, self.bb_callback, queue_size=1
        )

        # Published topics
        self.bounding_boxes_topic = "/ros_3d_bb/"
        self.marker_topic = "visualization_marker"

        # Publishers
        # ---

        if VERBOSE:
            rospy.loginfo("Subscribed to topic: " + self.bb_point_topic)

    def update_framerate(self):
        """Updates the program's framerate for making accurate predictions"""

        current_time = time.time()
        self.framerate = 1 / (current_time - self.time)
        self.time = current_time

    def bb_callback(self, bb_array):
        if TIME:
            timer = Timer("update")
            self.timers.append(timer)

        # Publishing the coordinate frames 
        self.tf_publisher.publish()
        self.tf_publisher_world.publish()

        # Finding the transformation from the world to the RealSense camera,
        # as the detected objects should be situated in the world frame.
        transform = self.tf_buffer.lookup_transform(
            self.frame_id_world, self.frame_id_realsense, rospy.Time()
        )

        if DEBUG:
            rospy.loginfo(transform)

        self.bounding_boxes = []
        for bounding_box in bb_array.boxes:
            # "do_transform_pose" requires stamped poses, so converting the original to stamped
            original_pose_stamped = PoseStamped(
                bounding_box.header, bounding_box.center)
            transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose(
                original_pose_stamped, transform)
            # Don't transform the size vector, as the pose is already transformed!

            new_bb = BoundingBox3D(
                bounding_box.header, transformed_pose_stamped.pose, bounding_box.size)

            if VERBOSE:
                rospy.loginfo(new_bb)

            self.bounding_boxes.append(
                BoundingBox(bounding_box=new_bb)
            )

        if TIME:
            timer.update()

        self.update_framerate()

        if TIME:
            timer.update()

        self.tracker.update(self.bounding_boxes)

        if TIME:
            timer.update()

        self.predictor.update()

        if TIME:
            timer.update()

        self.predictor.predict(self.framerate * 2)

        if TIME:
            timer.update()

        detections = self.tracker.objects

        # Visualization
        for obj in detections:
            # scaling = 1000  # From mm to m
            scaling = 1
            x = obj.x / scaling
            y = obj.y / scaling
            height = obj.height / scaling
            diameter = obj.diameter / scaling
            rospy.loginfo("Diameter:" + str(diameter))
            v_x = x + obj.v_x / scaling * self.framerate
            v_y = y + obj.v_y / scaling * self.framerate
            duration = self.max_frames_disappeared / self.framerate
            # Send data from the tracker
            self.rviz.text(obj.uid, x, y, duration=duration)
            self.rviz.cylinder(obj.uid, x, y, height, diameter,
                               duration=duration, alpha=0.5)
            self.rviz.arrow(obj.uid, x, y, v_x, v_y, duration=duration)
            # Send data from the predictor
            # Only the x and y
            predicted_x, predicted_y,  = self.predictor.predictions[obj.uid][:2]
            predicted_x /= scaling
            predicted_y /= scaling
            self.rviz.arrow(obj.uid + 1000, x, y, predicted_x,
                            predicted_y, duration=duration, r=0, g=1, b=0)

        self.rviz.publish()

        if TIME:
            timer.stop()

    def shutdown(self):
        if TIME:
            # for timer in self.timers:
            #     rospy.loginfo(timer)
            averages = Timer.average_times(self.timers)
            rospy.loginfo("Average timings (in ms): " + str(averages))
        rospy.loginfo("Exiting...")


def main():
    rospy.init_node("ros_3d_bb_tracker")
    module = RosTracker()
    rospy.on_shutdown(module.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
