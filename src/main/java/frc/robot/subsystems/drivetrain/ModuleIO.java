package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {

        public double drivePosition = 0;
        public double driveVelocity = 0;
        public double driveAppliedVolts = 0;
        public double[] driveCurrentAmps = new double[] {};
        public double[] driveTempCelcius = new double[] {};

        public double steerAngle = 0;
        public double steerAppliedVolts = 0;
        public double[] steerCurrentAmps = new double[] {};
        public double[] steerTempCelcius = new double[] {};

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
