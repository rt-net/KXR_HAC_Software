from motion_control.motion_control_library import MotionLibrary

MOTION = MotionLibrary()

while True:
    MOTION.plot_set()
    MOTION.walk_forward(90)
    MOTION.turn(90)
    MOTION.walk_sideway(90)