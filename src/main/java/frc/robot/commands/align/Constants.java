package frc.robot.commands.align;

public class Constants {
    // Angle tolerance in degrees. This should be as small as possible while
    // still allowing the robot to finish rotating.
    public static final double ROTATION_ALIGNMENT_TOLERANCE = 3.0;

    // Left/right tolerance in meters. This should be as small as possible while
    // still allowing the robot to align positionally.
    public static final double PRECISE_ALIGNMENT_POSITION_TOLERANCE = 0.03;

    // Distance from the center of the reef wall to either pole
    public static final double REEF_POLE_CENTER_OFFSET = 0.1558544;

    // Rough alignment constants
    public static final double ROUGH_ALIGN_TARGET_REEF_DISTANCE = 0.45;
}
