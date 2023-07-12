from motion_control.motion_control_library import MotionLibrary

MOTION = MotionLibrary()
MOTION.set_plot()
while True:
    MOTION.walk_forward(90)
    MOTION.turn(90)
    MOTION.walk_sideway(90)