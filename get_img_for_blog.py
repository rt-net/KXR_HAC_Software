import detect_ball_for_blog
import detect_edge
import cv2

delay = 1

while True:
    
    resultimg, a, b, c = detect_edge.main()
    
    cv2.imshow("frame", resultimg)
    
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break
    
    print(a)