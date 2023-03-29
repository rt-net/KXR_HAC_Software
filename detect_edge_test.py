import detect_edge_2
import detect_edge
import get_distance_from_the_edge
import global_value as g
import cv2

delay = 1


g.framex = 345
g.framey = 395

while True:
    a,b = detect_edge_2.main()
    
    resultimg, x, y, z = detect_edge.main()

    print(a, b)
    
    cv2.imshow("frame", resultimg)
    
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break
    
    if a != 0:
        dist = get_distance_from_the_edge.main(a, b)
        print(dist)

