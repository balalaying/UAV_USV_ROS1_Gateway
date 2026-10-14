#!/usr/bin/env python3

import rospy
import json
import cv2
import traceback
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge


class TrackingVisualizerNode:


    def __init__(self):

        rospy.init_node(
            "tracking_visualizer_node"
        )

        self.bridge = CvBridge()

        self.tracks=[]


        rospy.Subscriber(
            "/fleet/uplink/usv_01/camera",
            Image,
            self.image_callback,
            queue_size=1
        )


        rospy.Subscriber(
            "/perception/usv_01/camera/tracks",
            String,
            self.track_callback,
            queue_size=1
        )


        self.pub=rospy.Publisher(
            "/perception/usv_01/camera/tracks/image",
            Image,
            queue_size=1
        )


        rospy.loginfo(
            "Tracking visualizer started"
        )


    def track_callback(self,msg):

        try:
            self.tracks=json.loads(
                msg.data
            )

        except:

            self.tracks=[]



    def image_callback(self,msg):

        try:

            img=self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough"
            )


            img=np.array(
                img,
                copy=True
            )


            # 转显示格式
            if img.dtype != np.uint8:

                img=cv2.convertScaleAbs(
                    img
                )


            if len(img.shape)==2:

                img=cv2.cvtColor(
                    img,
                    cv2.COLOR_GRAY2BGR
                )


            for t in self.tracks:


                bbox=t.get(
                    "bbox",
                    []
                )

                if len(bbox)!=4:
                    continue


                x1,y1,x2,y2=[
                    int(v)
                    for v in bbox
                ]


                tid=t.get(
                    "track_id",
                    -1
                )

                conf=t.get(
                    "confidence",
                    0
                )


                cv2.rectangle(
                    img,
                    (x1,y1),
                    (x2,y2),
                    (0,255,0),
                    2
                )


                cv2.putText(
                    img,
                    "ID:{} {:.2f}".format(
                        tid,
                        conf
                    ),
                    (x1,y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0,255,0),
                    2
                )



            # 不使用cv2_to_imgmsg
            output=Image()

            output.header=msg.header

            output.height=img.shape[0]

            output.width=img.shape[1]

            output.encoding="bgr8"

            output.is_bigendian=False

            output.step=img.shape[1]*3

            output.data=np.array(
                img,
                dtype=np.uint8
            ).tobytes()


            self.pub.publish(
                output
            )


        except Exception:

            rospy.logerr(
                traceback.format_exc()
            )



if __name__=="__main__":

    TrackingVisualizerNode()

    rospy.spin()

