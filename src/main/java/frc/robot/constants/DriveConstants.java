package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public final double kTrackWidth = Units.inchesToMeters(22.75);
    public final double kMaxSpeed = 6380.0 / 60.0 / 6.75 * Units.inchesToMeters(4) * Math.PI;
    public final double kMaxAngularSpeed = 2 * Math.PI;

    public final int kPigeon = -1;

    public final int kDriveFrontLeft = 31;
    public final int kSteerFrontLeft = 14;
    public final int kEncoderFrontLeft = 0;
    public final double kSteerOffsetFrontLeft = 0.0; // FIXME Measure and set front left steer offset

    public final int kDriveFrontRight = 12; 
    public final int kSteerFrontRight = 17;
    public final int kEncoderFrontRight = 1; 
    public final double kSteerOffsetFrontRight = 0.0; // FIXME Measure and set front right steer offset

    public final int kDriveBackLeft = 7; 
    public final int kSteerBackLeft = 8;
    public final int kEncoderBackLeft = 9; 
    public final double kSteerOffsetBackLeft = 0.0; // FIXME Measure and set back left steer offset

    public final int kDriveBackRight = 10;
    public final int kSteerBackRight = 11;
    public final int kEncoderBackRight = 12; 
    public final double kSteerOffsetBackRight = 0.0; // FIXME Measure and set back right steer offset
}
