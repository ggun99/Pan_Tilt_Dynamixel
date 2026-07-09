import cv2 as cv
import numpy as np
import os

class FaceRecognizer:
    def __init__(self): #, model_path):

        self.cam = cv.VideoCapture(4)
        self.model = os.path.join("res10_300x300_ssd_iter_140000_fp16.caffemodel")
        self.config = os.path.join("deploy.prototxt")  # Placeholder for config
        self.net = cv.dnn.readNetFromCaffe(self.config, self.model)

    def recognize_face(self, face_image):
        gray_face = cv.cvtColor(face_image, cv.COLOR_BGR2RGB)
        label, confidence = self.model.predict(gray_face)
        return label, confidence

def main():
    # model_path = "face_model.xml"
    face_recognizer = FaceRecognizer()

    while True:
        ret, frame = face_recognizer.cam.read()
        if not ret:
            break
        h,w = frame.shape[:2]
        blob = cv.dnn.blobFromImage(cv.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0),False, False)

        face_recognizer.net.setInput(blob)
        detections = face_recognizer.net.forward()

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.6:  
                box = detections[0, 0, i, 3:7] * [w, h, w, h]
                (x, y, x2, y2) = box.astype("int")

                cv.rectangle(frame, (x, y), (x2, y2), (0, 255, 0), 2)

        cv.imshow("Frame", frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    face_recognizer.cam.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()