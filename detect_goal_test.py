from vision.vision_library import VisionLibrary
import cv2

VISION = VisionLibrary()

delay = 1

while True:
    frame_mask, frame = VISION.detect_goal()
    cv2.imshow("frame1", frame_mask)
    cv2.imshow("frame2", frame)

    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break