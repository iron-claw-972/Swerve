package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    
    public final double kTrackWidth = Units.inchesToMeters(22.75);
    
    public final double kGearRatio = 6.75;
    public final double kGearRatioSteer = 0.07;
    public final double kWheelRadius = Units.inchesToMeters(2);

    public final double kMaxSpeed = 6380.0 / 60.0 / kGearRatio * kWheelRadius * 2 * Math.PI;
    public final double kMaxAngularSpeed = 2 * Math.PI;

    public final int kPigeon = 13;

    public final int kDriveFrontLeft = 1;
    public final int kSteerFrontLeft = 2;
    public final int kEncoderFrontLeft = 3;
    public final double kSteerOffsetFrontLeft = -180.0; // FIXME Measure and set front left steer offset

    public final int kDriveFrontRight = 4; 
    public final int kSteerFrontRight = 5;
    public final int kEncoderFrontRight = 6; 
    public final double kSteerOffsetFrontRight = -180.0; // FIXME Measure and set front right steer offset

    public final int kDriveBackLeft = 7; 
    public final int kSteerBackLeft = 8;
    public final int kEncoderBackLeft = 9; 
    public final double kSteerOffsetBackLeft = -180.0; // FIXME Measure and set back left steer offset

    public final int kDriveBackRight = 10;
    public final int kSteerBackRight = 11;
    public final int kEncoderBackRight = 12; 
    public final double kSteerOffsetBackRight = -180.0; // FIXME Measure and set back right steer offset

    // PID
    // Drive
    public final double kDriveP = 0.75;
    public final double kDriveI = 0;
    public final double kDriveD = 0;
    // Steer
    public final double kSteerP = 0.2;
    public final double kSteerI = 0;
    public final double kSteerD = 0;

    // FEEDFORWARD
    // Drive
    public final double kDriveKS = 0;
    public final double kDriveKV = 0;
    // Steer
    public final double kSteerKS = 0;
    public final double kSteerKV = 0;
}
