package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DrivetrainIO {

    public static class DrivetrainIOInputs implements LoggableInputs {
        
        public double frontLeft_steerAngle = 0;
        public double frontLeft_driveVelocity = 0;

        public double frontRight_steerAngle = 0;
        public double frontRight_driveVelocity = 0;

        public double backLeft_steerAngle = 0;
        public double backLeft_driveVelocity = 0;

        public double backRight_steerAngle = 0;
        public double backRight_driveVelocity = 0;

        @Override
        public void toLog(LogTable table) {
            table.put("frontLeft_steerAngle", frontLeft_steerAngle);
            table.put("frontLeft_driveVelocity", frontLeft_driveVelocity);
        }

        @Override
        public void fromLog(LogTable table) {
            frontLeft_steerAngle = table.getDouble("frontLeft_steerAngle", 0);
            frontLeft_driveVelocity = table.getDouble("frontLeft_driveVelocity", 0);
        }

    }

    public void updateInputs(DrivetrainIOInputs inputs);
    public default void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {}
    public default void updateOdometry() {}

    public default void simulatePeriodic() {}
}