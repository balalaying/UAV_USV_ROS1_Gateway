#!/usr/bin/env python3


from ultralytics import YOLO

from usv_camera_detection_node import VisualCandidate

from vision_guided_core import (
    AFFILIATION_UNKNOWN,
)


class YoloDetectorBackend:


    def __init__(self):

        self.model = YOLO(
            "/home/p/yolov8n.pt"
        )

        print(
            "YOLO backend initialized"
        )


    def detect(self, image):

        candidates = []

        results = self.model(
            image,
            verbose=False
        )


        for result in results:

            boxes = result.boxes

            for box in boxes:

                cls = int(
                    box.cls[0]
                )

                conf = float(
                    box.conf[0]
                )


                # YOLO COCO:
                # boat = 8
                if cls != 8:
                    continue


                x1,y1,x2,y2 = (
                    box.xyxy[0]
                    .cpu()
                    .numpy()
                    .astype(int)
                )


                candidates.append(
                    VisualCandidate(
                        rectangle=(
                            x1,
                            y1,
                            x2,
                            y2
                        ),
                        class_name="vessel",
                        class_confidence=conf,
                        affiliation=AFFILIATION_UNKNOWN,
                        affiliation_confidence=0.0,
                        marker_name="yolo"
                    )
                )


        return candidates