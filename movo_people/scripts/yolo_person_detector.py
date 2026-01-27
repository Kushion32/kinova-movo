from ultralytics import YOLO
import cv2


class YoloPersonDetector:
    """
    Wraps YOLOv8 inference and draws person detections on BGR images.
    """

    def __init__(self, model_path="yolov8n.pt", conf=0.25):
        self.model = YOLO(model_path)
        self.conf = conf

    def detect_people(self, image):
        """
        Runs YOLOv8 on the provided BGR image and returns the person count.

        :param image: OpenCV BGR image.
        :returns: (count, annotated_image, list_of_boxes)
        """
        results = self.model.predict(source=image, conf=self.conf, verbose=False)[0]

        annotated = image.copy()
        person_boxes = []
        for box in results.boxes:
            cls = int(box.cls[0])
            if cls == 0:  # person class in COCO
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                person_boxes.append((x1, y1, x2, y2))
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated,
                    "person",
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

        return len(person_boxes), annotated, person_boxes
