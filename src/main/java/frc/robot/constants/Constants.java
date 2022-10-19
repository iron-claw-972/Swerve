package frc.robot.constants;

public final class Constants {
    public static final double GRAVITATIONAL_ACCEL = 9.8;
    public static final double kMaxVoltage = 12.0;
    public static final double kEncoderResolution = 2048;

    // CAN bus names
    public static final String kCanivoreCAN = "Canivore";
    public static final String kRioCAN = "rio";

    public static final OIConstants oi = new OIConstants();
    public static final DriveConstants drive = new DriveConstants();
    public static final AutoConstants auto = new AutoConstants();
}
