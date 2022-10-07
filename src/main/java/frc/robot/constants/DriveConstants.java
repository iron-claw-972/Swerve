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

    public final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 21;
    public final int FRONT_LEFT_MODULE_STEER_MOTOR = 17;
    public final int FRONT_LEFT_MODULE_STEER_ENCODER = -1; // FIXME Set front left steer encoder ID
    public final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front left steer offset

    public final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 0; // FIXME Set front right drive motor ID
    public final int FRONT_RIGHT_MODULE_STEER_MOTOR = 0; // FIXME Set front right steer motor ID
    public final int FRONT_RIGHT_MODULE_STEER_ENCODER = -1; // FIXME Set front right steer encoder ID
    public final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front right steer offset

    public final int BACK_LEFT_MODULE_DRIVE_MOTOR = 27; 
    public final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
    public final int BACK_LEFT_MODULE_STEER_ENCODER = -1; // FIXME Set back left steer encoder ID
    public final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back left steer offset

    public final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 29;
    public final int BACK_RIGHT_MODULE_STEER_MOTOR = 26;
    public final int BACK_RIGHT_MODULE_STEER_ENCODER = -1; // FIXME Set back right steer encoder ID
    public final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back right steer offset
}
