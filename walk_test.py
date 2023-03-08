import walk_sideway
import global_value as g
import get_body_angle
import turn
import walk_forward


BodyAngle = get_body_angle.main() #IMUの価を読み込む
print(BodyAngle)

# walk_sideway.main(-200)
walk_forward.main(150)

print(g.X, g.Y)