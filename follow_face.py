import cv2 as cv
from pid_dynamixel import PIDControl
from detect_face import FaceRecognizer


class FollowFace:
    def __init__(self, pid_controller: PIDControl, face_recognizer: FaceRecognizer):
        self.pid_controller = pid_controller
        self.face_recognizer = face_recognizer


    def run(self):
        self.pid_controller.initialize()
        try:
            while True:
                ret, frame = self.face_recognizer.cam.read()
                if not ret:
                    break

                _, selected_face = self.face_recognizer.detect_faces(frame)
                if selected_face is not None and self.face_recognizer.selected_face_center is not None:
                    h, w = frame.shape[:2]
                    face_center_x, face_center_y = self.face_recognizer.selected_face_center
                    self.pid_controller.update(w, h, face_center_x, face_center_y)

                    x1, y1, x2, y2 = self.face_recognizer.selected_face_box
                    cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)

                cv.imshow("Follow Face", frame)
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.face_recognizer.cam.release()
            self.pid_controller.close()
            cv.destroyAllWindows()

if __name__ == "__main__":
    # Initialize PID controller and face recognizer
    pid_controller = PIDControl()
    face_recognizer = FaceRecognizer()

    # Create FollowFace instance and run
    follow_face = FollowFace(pid_controller, face_recognizer)
    follow_face.run()