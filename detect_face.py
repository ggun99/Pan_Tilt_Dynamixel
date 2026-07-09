import cv2 as cv
import numpy as np
import os

class FaceRecognizer:
    def __init__(self): #, model_path):

        self.cam = cv.VideoCapture(4)
        self.model = os.path.join("res10_300x300_ssd_iter_140000_fp16.caffemodel")
        self.config = os.path.join("deploy.prototxt")
        self.net = cv.dnn.readNetFromCaffe(self.config, self.model)
        self.selected_face = None
        self.selected_face_box = None
        self.selected_face_center = None
        self.selected_before = None

    def recognize_face(self, face_image):
        gray_face = cv.cvtColor(face_image, cv.COLOR_BGR2RGB)
        label, confidence = self.model.predict(gray_face)
        return label, confidence

    def detect_faces(self, frame, confidence_threshold=0.6):
        h, w = frame.shape[:2]
        blob = cv.dnn.blobFromImage(
            cv.resize(frame, (300, 300)),
            1.0,
            (300, 300),
            (104.0, 177.0, 123.0),
            False,
            False,
        )

        self.net.setInput(blob)
        detections = self.net.forward()

        face_candidates = []
        frame_center_x = w * 0.5
        frame_center_y = h * 0.5

        for i in range(detections.shape[2]):
            confidence = float(detections[0, 0, i, 2])
            if confidence < confidence_threshold:
                continue

            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            x1, y1, x2, y2 = box.astype("int")
            face_center_x = int((x1 + x2) / 2)
            face_center_y = int((y1 + y2) / 2)
            distance = (face_center_x - frame_center_x) ** 2 + (face_center_y - frame_center_y) ** 2

            face_candidates.append(
                {
                    "index": i,
                    "confidence": confidence,
                    "box": (x1, y1, x2, y2),
                    "center": (face_center_x, face_center_y),
                    "distance_to_frame_center": distance,
                }
            )

        if face_candidates:
            if self.selected_before is None:
                selected_face = min(face_candidates, key=lambda item: item["distance_to_frame_center"])
            else:
                previous_center_x, previous_center_y = self.selected_before["center"]
                selected_face = min(
                    face_candidates,
                    key=lambda item: (
                        item["center"][0] - previous_center_x
                    ) ** 2 + (
                        item["center"][1] - previous_center_y
                    ) ** 2,
                )

            self.selected_face = selected_face["index"]
            self.selected_face_box = selected_face["box"]
            self.selected_face_center = selected_face["center"]
            self.selected_before = selected_face
        else:
            self.selected_face = None
            self.selected_face_box = None
            self.selected_face_center = None

        return face_candidates, self.selected_face

def main():
    # model_path = "face_model.xml"
    face_recognizer = FaceRecognizer()

    while True:
        ret, frame = face_recognizer.cam.read()
        if not ret:
            break

        face_candidates, selected_face = face_recognizer.detect_faces(frame)

        if selected_face is not None:
            x1, y1, x2, y2 = face_recognizer.selected_face_box
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            face_center_x, face_center_y = face_recognizer.selected_face_center
            cv.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)

        cv.imshow("Frame", frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    face_recognizer.cam.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()