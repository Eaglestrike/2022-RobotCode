TODOLIST:

1. Verify polarities at interface boundaries
Consult the coordinate frame I added to Constants.h. All functinality should be abstracted so as to appeal to this standard.

Do this from the bottom-up, starting with swerve modules.

All swerve modules when commanded +power, 0rad, should drive the robot in the +x direction, with speed sensor values increasing.

When that is done, you adjust the negation of arguments in SwerveDrive::Drive to restore driving.

When that is done, SwerveDrive::Drive should take x, y, and a bool indicating whether the x and y are given in world frame or robot frame. And the polarities must match. If this breaks the orientation of the Joysticks, you multiply the joystick values first, because this is not a swerve-drive concern, its a joystick concern. This ensures that the path following code can make use of Drive() in a sane way.

2. Tune Odometry

See github comment for this. Verify that this works by driving around all crazy. It won't work very well, (it never does), but it should WORK. It has to be passable for 10 seconds. Over that sort of time range, making smooth, medium speed manuevres, I'd expect at most 1-2 feet of error.

3. Tune translational controllers.

Setup an automode where the Robot drives a purely translational trajectory, in solely x, with 0 initial and end velocity. Begin by setting P=D=0. You can empirically derive an initial value for F using the theoretical max speed of the robot:

1.0 = TRAJ_TRANSLATE_F * ROBOT_MAX_SPEED

Your final value of F may be higher or lower than this. Tune F until the robot follows the path pretty well on it's own. The robot should arrive in about the right amount of time, but probably won't stop accurately. Then add in P. Then add in D (D should be negative, and very very small, if you use it at all). Remember that the purpose of adding D in is to allow a higher P, ending in faster convergence, so you can and should tune P too high and then add a small D.

The usable gains for x and y should be the same. Test the tuned result on a path in both x and y.

4. Tune rotational controllers.

Do the same procedure as above on a path with only a rotational component.

Then try a path with all components.

5. Make some auto modes
ur done! See the stuff I checked in for roughly how I'd do it.
