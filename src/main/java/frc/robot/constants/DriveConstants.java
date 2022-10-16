package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public final double KTrackWidth = Units.inchesToMeters(22.75);
    public final double kMaxSpeed = 6380.0 / 60.0 / 6.75 * Units.inchesToMeters(4) * Math.PI;

    public final int kPigeon = -1;

    public final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 31;
    public final int FRONT_LEFT_MODULE_STEER_MOTOR = 14;
    public final int FRONT_LEFT_MODULE_STEER_ENCODER = 0;
    public final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front left steer offset

    public final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12; 
    public final int FRONT_RIGHT_MODULE_STEER_MOTOR = 17;
    public final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1; 
    public final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front right steer offset

    public final int BACK_LEFT_MODULE_DRIVE_MOTOR = 18; 
    public final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
    public final int BACK_LEFT_MODULE_STEER_ENCODER = 2; 
    public final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back left steer offset

    public final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5;
    public final int BACK_RIGHT_MODULE_STEER_MOTOR = 2;
    public final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; 
    public final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back right steer offset
}
