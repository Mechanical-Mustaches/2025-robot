package frc.robot.commands.align;

public class Constants {
    public enum Mode {
        LEFT,
        RIGHT
    }
    // TODO: Tune all of these values

    // Angle tolerance in degrees. This should be as small as possible while
    // still allowing the robot to finish rotating.
    public static final double ROTATION_ALIGNMENT_TOLERANCE = 1.5;

    // Target distance from the reef to the center of the robot for precise
    // alignment. This command likely ends before this is reached because we
    // will lose april tag vision this close up.
    public static final double PRECISE_ALIGNMENT_FORWARD_SETPOINT = 0.47;
    // Left/right tolerance in meters. This should be as small as possible while
    // still allowing the robot to align positionally.
    public static final double PRECISE_ALIGNMENT_POSITION_TOLERANCE = 0.03;

    // Distance from the center of the reef wall to either pole
    public static final double REEF_POLE_CENTER_OFFSET = 0.1558544;

    // Distance away from the wall to path to. This should be far enough to see
    // april tags with a good FOV, but as close as possible to reduce alignment
    // time.
    public static final double ROUGH_ALIGN_TARGET_REEF_DISTANCE = 0.65;
}
