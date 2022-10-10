package frc.robot.subsystems.unused;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;

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

            table.put("frontRight_steerAngle", frontRight_steerAngle);
            table.put("frontRight_driveVelocity", frontRight_driveVelocity);

            table.put("backLeft_steerAngle", backLeft_steerAngle);
            table.put("backLeft_driveVelocity", backLeft_driveVelocity);

            table.put("backRight_steerAngle", backRight_steerAngle);
            table.put("backRight_driveVelocity", backRight_driveVelocity);
        }

        @Override
        public void fromLog(LogTable table) {
            frontLeft_steerAngle = table.getDouble("frontLeft_steerAngle", 0);
            frontLeft_driveVelocity = table.getDouble("frontLeft_driveVelocity", 0);

            frontRight_steerAngle = table.getDouble("frontRight_steerAngle", 0);
            frontRight_driveVelocity = table.getDouble("frontRight_driveVelocity", 0);

            backLeft_steerAngle = table.getDouble("backLeft_steerAngle", 0);
            backLeft_driveVelocity = table.getDouble("backLeft_driveVelocity", 0);

            backRight_steerAngle = table.getDouble("backRight_steerAngle", 0);
            backRight_driveVelocity = table.getDouble("backRight_driveVelocity", 0);

        }

    }

    public void updateInputs(DrivetrainIOInputs inputs);
    public default void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {}
    public default void updateOdometry() {}
    public default Pose2d getOdometry() {
        return null;
    }

    public default void simulationPeriodic() {}
}