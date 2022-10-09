package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.Constants;

public class SwerveModule {
    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonFX m_steerMotor;

    private final TalonEncoder m_driveEncoder;
    private final DutyCycleEncoder m_absEncoder;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController =
        new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.swerve.kMaxAngularSpeed, 2 * Math.PI));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 0);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0);

    public SwerveModule(
        int driveMotorPort,
        int steerMotorPort,
        int absEncoderPort) {
        m_driveMotor = new WPI_TalonFX(driveMotorPort);
        m_steerMotor = new WPI_TalonFX(steerMotorPort);

        m_driveEncoder = new TalonEncoder(m_driveMotor);
        m_absEncoder = new DutyCycleEncoder(absEncoderPort);

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
        return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(m_steerMotor.get()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_steerMotor.get()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
            m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
            m_turningPIDController.calculate(m_steerMotor.get(), state.angle.getRadians());

        final double turnFeedforward =
            m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_steerMotor.setVoltage(turnOutput + turnFeedforward);
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

    /**
     * Returns the drive motor in a swerve module.
     *
     * @return The WPI_TalonFX swerve drive motor.
     */
    public WPI_TalonFX getDriveMotor() {
        return m_driveMotor;
    }

    /**
     * Returns the turning motor in a swerve module.
     *
     * @return The WPI_TalonFX swerve turning motor.
     */
    public WPI_TalonFX getSteerMotor() {
        return m_steerMotor;
    }

    /**
     * Returns the drive encoder in a swerve module.
     *
     * @return The TalonEncoder swerve drive encoder.
     */
    public TalonEncoder getDriveEncoder() {
        return m_driveEncoder;
    }
}