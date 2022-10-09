package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import ctre_shims.TalonEncoder;
import ctre_shims.TalonEncoderSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import frc.robot.constants.Constants;

public class SwerveModuleSim {
    
    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonFX m_steerMotor;

    private final TalonEncoder m_driveEncoder;
    private final DutyCycleEncoder m_absEncoder;

    private final TalonEncoderSim m_driveEncoderSim;
    private final DutyCycleEncoderSim m_absEncoderSim;

    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    private final ProfiledPIDController m_turningPIDController =
        new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.swerve.kMaxAngularSpeed, 2 * Math.PI));

    public final FlywheelSim m_steerMotorSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(Constants.swerve.kvVoltSecondsPerMeter, Constants.swerve.kaVoltSecondsSquaredPerMeter),
        DCMotor.getFalcon500(1),
        Constants.swerve.kSteerGearRatio
    );

    public final FlywheelSim m_driveMotorSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(Constants.swerve.kvVoltSecondsPerMeter, Constants.swerve.kaVoltSecondsSquaredPerMeter),
        DCMotor.getFalcon500(1),
        Constants.swerve.kDriveGearRatio
    );

    private double m_driveOutput;
    private double m_turnOutput;

    private double m_simTurnEncoderDistance;
    private double m_simDriveEncoderDistance;

    public SwerveModuleSim(
        int driveMotorPort,
        int steerMotorPort,
        int absEncoderPort) {
        m_driveMotor = new WPI_TalonFX(driveMotorPort);
        m_steerMotor = new WPI_TalonFX(steerMotorPort);

        m_driveEncoder = new TalonEncoder(m_driveMotor);
        m_absEncoder = new DutyCycleEncoder(absEncoderPort);

        m_driveEncoderSim = new TalonEncoderSim(new TalonEncoder(m_driveMotor));
        m_absEncoderSim = new DutyCycleEncoderSim(m_absEncoder);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveEncoder.setDistancePerPulse(2 * Math.PI * Constants.swerve.kWheelRadius / 6.75 / Constants.kEncoderResolution);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        m_absEncoder.setDistancePerRotation(2 * Math.PI);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_absEncoder.getDistance()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(m_absEncoder.getDistance()));

        // Calculate the drive output from the drive PID controller.
        m_driveOutput =
            m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        m_turnOutput =
            m_turningPIDController.calculate(m_absEncoder.getDistance(), state.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        m_driveMotor.set(m_driveOutput);
        m_steerMotor.set(m_turnOutput);
    }

    /** Zeros all the SwerveModule encoders. */
    public void resetEncoders() {
        m_driveEncoder.reset();
        m_absEncoder.reset();
    }

    /** Simulate the SwerveModule */
    public void simulationPeriodic(double dt) {
        m_steerMotorSim.setInputVoltage(m_turnOutput / Constants.swerve.kMaxAngularSpeed * RobotController.getBatteryVoltage());
        m_driveMotorSim.setInputVoltage(m_driveOutput / Constants.drive.kMaxSpeed * RobotController.getBatteryVoltage());

        m_steerMotorSim.update(dt);
        m_driveMotorSim.update(dt);

        // Calculate distance traveled using RPM * dt
        m_simTurnEncoderDistance += m_steerMotorSim.getAngularVelocityRadPerSec() * dt;
        m_absEncoderSim.setDistance(m_simTurnEncoderDistance);
        m_absEncoderSim.set(m_steerMotorSim.getAngularVelocityRadPerSec());

        // m_simDriveEncoderDistance += m_driveMotorSim.getAngularVelocityRadPerSec() * dt;
        // m_driveEncoderSim.setDistance(m_simDriveEncoderDistance);
        m_driveEncoderSim.setRate(m_driveMotorSim.getAngularVelocityRadPerSec());

        System.out.println("Module: " + m_steerMotor.getDeviceID() + " " + m_steerMotorSim.getAngularVelocityRadPerSec() + " " + getState());
    }

    /**
     * Returns the current angle of the steer motor.
     *
     * @return The current angle of the steer motor.
     */
    public double getSteerAngle() {
        return new Rotation2d(m_steerMotor.get()).getDegrees();
    }

    /**
     * Returns the current velocity of the drive motor.
     *
     * @return The current velocity of the drive motor.
     */
    public double getDriveVelocity() {
        return m_driveMotor.getSelectedSensorVelocity();
    }


}
