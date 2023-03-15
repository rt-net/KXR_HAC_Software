import detect_edge_2
import get_distance_from_the_edge
import global_value as g


g.framex = 345
g.framey = 395

while True:
    a,b = detect_edge_2.main()

    print(a, b)
    
    if a != 0:
        dist = get_distance_from_the_edge.main(a, b)
        print(dist)

