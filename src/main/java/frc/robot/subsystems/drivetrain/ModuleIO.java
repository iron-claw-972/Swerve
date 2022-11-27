package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {

    public static class ModuleIOInputs implements LoggableInputs {

        public double drivePosition = 0;
        public double driveVelocity = 0;
        public double driveAppliedVolts = 0;
        public double[] driveCurrentAmps = new double[] {};
        public double[] driveTempCelcius = new double[] {};

        public double steerAngle = 0;
        public double steerAppliedVolts = 0;
        public double[] steerCurrentAmps = new double[] {};
        public double[] steerTempCelcius = new double[] {};

        @Override
        public void toLog(LogTable table) {
            table.put("driveVelocity", driveVelocity);
            table.put("driveAppliedVolts", driveAppliedVolts);
            table.put("driveCurrentAmps", driveCurrentAmps);
            table.put("driveTempCelcius", driveTempCelcius);

            table.put("steerAngle", steerAngle);
            table.put("steerAppliedVolts", steerAppliedVolts);
            table.put("steerCurrentAmps", steerCurrentAmps);
            table.put("steerTempCelcius", steerTempCelcius);
        }

        @Override
        public void fromLog(LogTable table) {
            driveVelocity = table.getDouble("driveVelocity", driveVelocity);
            driveAppliedVolts = table.getDouble("driveAppliedVolts", driveAppliedVolts);
            driveCurrentAmps = table.getDoubleArray("driveCurrentAmps", driveCurrentAmps);
            driveTempCelcius = table.getDoubleArray("driveTempCelcius", driveTempCelcius);

            steerAngle = table.getDouble("steerAngle", steerAngle);
            steerAppliedVolts = table.getDouble("steerAppliedVolts", steerAppliedVolts);
            steerCurrentAmps = table.getDoubleArray("steerCurrentAmps", steerCurrentAmps);
            steerTempCelcius = table.getDoubleArray("steerTempCelcius", steerTempCelcius);
        }

    }

    public void updateInputs(ModuleIOInputs inputs);

    public default SwerveModuleState getState() {
        return null;
    }

    public void setDesiredState(SwerveModuleState swerveModuleState);

    public PIDController getDrivePID();

    public ProfiledPIDController getSteerPID();

    public SimpleMotorFeedforward getDriveFF();

    public SimpleMotorFeedforward getSteerFF();

}
