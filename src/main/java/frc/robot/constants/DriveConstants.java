package frc.robot.constants;

public class DriveConstants {
  /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public final double TRACKWIDTH_METERS = 0.57785;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public final double WHEELBASE_METERS = TRACKWIDTH_METERS; 

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
