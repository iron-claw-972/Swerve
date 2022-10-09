package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class SwerveModuleConstants {
    public final double kMaxAngularSpeed = 2 * Math.PI;
    public final double kMaxAngularAccel = 2 * Math.PI;
    public final double kWheelRadius = Units.inchesToMeters(2);

    // TODO: Tune these (these are for robot swerve sim)
    public final double ksVolts = 0.587;
    public final double kvVoltSecondsPerMeter = 2.3;
    public final double kaVoltSecondsSquaredPerMeter = 0.0917;

    // TODO: fill these out, rn just random numbers or smth
    public final double kSteerGearRatio = 6.75;
    public final double kDriveGearRatio = 6.75;
}