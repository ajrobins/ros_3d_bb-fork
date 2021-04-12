#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import numpy as np
from scipy.spatial import distance


VERBOSE = False


class DetectedObject:
    def __init__(self, uid, x=0, y=0, v_x=0, v_y=0):
        self.x = x
        self.y = y
        self.v_x = v_x
        self.v_y = v_y
        self.uid = uid
        self.disappeared = 0
        print("Created new object:", self)

    def __eq__(self, other):
        return self.uid == other.uid

    def __hash__(self):
        return hash(self.uid)

    def __str__(self):
        return "UID: " + str(self.uid) + ", x: " + str(self.x) + ", y: " + str(self.y)

    def update(self, xy):
        x, y = xy
        """Updates the object's position, calculates the velocity, resets disappeared counter."""
        self.v_x = x - self.v_x
        self.v_y = y - self.v_y
        self.x = x
        self.y = y
        self.disappeared = 0

    def has_disappeared(self):
        self.disappeared += 1


class Tracker:
    def __init__(self, max_frames_disappeared=30):
        self.max_frames_disappeared = max_frames_disappeared
        self.next_uid = 0

        self.objects = []

    def new_object(self, xy):
        """Register a newly detected object."""
        self.objects.append(DetectedObject(self.next_uid, xy[0], xy[1]))

        # Increment the UID.
        self.next_uid += 1

    def get_coordinates(self):
        coordinates = []
        for detected_object in self.objects:
            # print("Object:", detected_object)
            coordinates.append((detected_object.x, detected_object.y))

        return coordinates

    def delete_if_disappeared(self, detected_object):
        """Delete the long-disappeared objects (>= max_frames_disappeared)"""
        if detected_object.disappeared >= self.max_frames_disappeared:
            self.objects.remove(detected_object)

    def update(self, points: list):
        """Updates the positions of detected objects, add new objects and deletes old objects."""
        print("Objects:", list(map(str, self.objects)))
        # print("len(points):", len(points))
        if len(points) == 0:
            for detected_object in self.objects:
                detected_object.disappeared()
                self.delete_if_disappeared(detected_object)
        elif len(self.objects) == 0:
            for point in points:
                # print("P:", point)
                self.new_object(point)
        else:
            # print("Points:", points)
            current_coordinates = self.get_coordinates()
            # print("---")
            distances = distance.cdist(
                np.array(current_coordinates), points, "euclidean")

            # The following part is from the following article:
            # https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/
            # Slightly modified the given code.
            rows = distances.min(axis=1).argsort()
            cols = distances.argmin(axis=1)[rows]
            # Basically, this finds indices with least distances between the frames

            used_rows = set()
            used_cols = set()

            for i, j in zip(rows, cols):
                if i in used_rows or j in used_cols:
                    continue
                else:
                    self.objects[i].update(points[j])
                    used_rows.add(i)
                    used_cols.add(j)

            len_rows = distances.shape[0]
            len_cols = distances.shape[1]

            if len_rows > len_cols:
                unused_rows = set(
                    range(0, distances.shape[0])).difference(used_rows)
                for i in unused_rows:
                    obj = self.objects[i]
                    print(obj, type(obj))
                    obj.has_disappeared()
                    self.delete_if_disappeared(obj)
            elif len_rows < len_cols:
                unused_cols = set(
                    range(0, distances.shape[1])).difference(used_cols)
                for j in unused_cols:
                    self.new_object(points[j])

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
        self.points = []
        self.tracker = Tracker(30)

        # Subscribed topics
        self.bb_point_topic = "/ros_3d_bb/point"

        # Subscribers
        self.bb_point_sub = rospy.Subscriber(
            self.bb_point_topic, Int32MultiArray, self.bb_point_callback, queue_size=1
        )

        if VERBOSE:
            rospy.loginfo("Subscribed to topic: " + self.bb_point_topic)

    def bb_point_callback(self, bb_point_multiarray):
        # bb_point_multiarray may contain zero to many points,
        # each described by 3 consecutive values.

        # Extract as many point x, y and z coordinates as found:
        nr_of_bb_points = len(bb_point_multiarray.data) // 3
        self.points = []
        for i in range(nr_of_bb_points):
            self.points.append(
                (
                    bb_point_multiarray.data[0 + 3 * i],
                    # bb_point_multiarray.data[1 + 3 * i],
                    bb_point_multiarray.data[2 + 3 * i],
                )
            )

        # print(self.points)
        self.tracker.update(self.points)

        if VERBOSE:
            rospy.loginfo(self.tracker.get_position_dict())

    def shutdown(self):
        print("Exiting...")


def main():
    rospy.init_node("ros_3d_bb_tracker")
    module = RosTracker()
    rospy.on_shutdown(module.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
