#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import supervision as sv

from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

import json


class ByteTrackTrackerNode:

    def __init__(self):

        rospy.init_node(
            "bytetrack_tracker_node"
        )

        self.vehicle_id = rospy.get_param(
            "~vehicle_id",
            "usv_01"
        )

        self.tracker = sv.ByteTrack()

        self.pub = rospy.Publisher(
            "/perception/{}/camera/tracks".format(
                self.vehicle_id
            ),
            String,
            queue_size=10
        )

        self.sub = rospy.Subscriber(
            "/perception/{}/camera/detections".format(
                self.vehicle_id
            ),
            Detection2DArray,
            self.callback,
            queue_size=1
        )


        rospy.loginfo(
            "ByteTrack tracker started"
        )


    def callback(self,msg):

        detections = []


        for det in msg.detections:

            if len(det.results)==0:
                continue


            bbox = det.bbox

            x1 = bbox.center.x - bbox.size_x/2
            y1 = bbox.center.y - bbox.size_y/2
            x2 = bbox.center.x + bbox.size_x/2
            y2 = bbox.center.y + bbox.size_y/2


            score = det.results[0].score


            detections.append(
                [
                    x1,
                    y1,
                    x2,
                    y2,
                    score
                ]
            )


        if len(detections)==0:
            return


        # supervision格式

        import numpy as np

        xyxy = np.array(
            [
                d[:4]
                for d in detections
            ]
        )

        confidence = np.array(
            [
                d[4]
                for d in detections
            ]
        )


        sv_detections = sv.Detections(
            xyxy=xyxy,
            confidence=confidence
        )


        tracked = self.tracker.update_with_detections(
            sv_detections
        )


        output=[]


        for i,box in enumerate(tracked.xyxy):

            item={

                "track_id":
                    int(tracked.tracker_id[i]),

                "bbox":
                    box.tolist(),

                "confidence":
                    float(tracked.confidence[i])

            }

            output.append(item)


        self.pub.publish(
            json.dumps(output)
        )


if __name__=="__main__":

    try:
        ByteTrackTrackerNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass