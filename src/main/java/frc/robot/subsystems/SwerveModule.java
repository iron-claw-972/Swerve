package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import ctre_shims.TalonEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.Constants;

public class SwerveModule {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_steerMotor;

  private final TalonEncoder m_driveEncoder;
  private final CANCoder m_encoder;

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
      int encoderPort) {
    m_driveMotor = new WPI_TalonFX(driveMotorPort, Constants.kCanivoreCAN);
    m_steerMotor = new WPI_TalonFX(steerMotorPort, Constants.kCanivoreCAN);

    m_driveEncoder = new TalonEncoder(m_driveMotor);
    m_encoder = new CANCoder(encoderPort, Constants.kCanivoreCAN);

    // reset encoder to factory defaults, reset position to the measurement of the absolute encoder
    // by default the CANcoder sets it's feedback coefficient to 0.087890625, to make degrees. 
    m_encoder.configFactoryDefault();
    m_encoder.setPositionToAbsolute();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(2 * Math.PI * Constants.swerve.kWheelRadius / 6.75 / Constants.kEncoderResolution);

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
    desiredState =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_encoder.getAbsolutePosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getRate(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_encoder.getAbsolutePosition(), desiredState.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_steerMotor.setVoltage(turnOutput + turnFeedforward);
  }
}