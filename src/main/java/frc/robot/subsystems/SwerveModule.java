package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import ctre_shims.TalonEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.constants.Constants;

public class SwerveModule {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_steerMotor;

  private final TalonEncoder m_driveEncoder;
  private final WPI_CANCoder m_encoder;

  private final PIDController m_drivePIDController = new PIDController(Constants.drive.kDriveP, Constants.drive.kDriveI,
      Constants.drive.kDriveD);

  private ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      Constants.drive.kSteerP,
      Constants.drive.kSteerI,
      Constants.drive.kSteerD,
      new TrapezoidProfile.Constraints(
          Constants.drive.kMaxAngularSpeed, Constants.drive.kMaxAngularAccel));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(Constants.drive.kDriveKS,
      Constants.drive.kDriveKV);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(Constants.drive.kSteerKS,
      Constants.drive.kSteerKV);

  public double m_offset = 0.0;

  public double driveOutput = 0;

  public SwerveModule(
      int driveMotorPort,
      int steerMotorPort,
      int encoderPort,
      double encoderOffset) {

    // s: 8.3
    // a: 8
    double altSpeed = 10;
    double altAcc = 12;


    if (encoderOffset == Constants.drive.kSteerOffsetFrontRight) {
      m_turningPIDController = new ProfiledPIDController(
      Constants.drive.kSteerP,
      Constants.drive.kSteerI,
      Constants.drive.kSteerD,
      new TrapezoidProfile.Constraints(Constants.drive.kMaxAngularSpeed, altAcc * 2 * Math.PI));
    }


    if (encoderOffset == Constants.drive.kSteerOffsetBackLeft) {
      m_turningPIDController = new ProfiledPIDController(
      Constants.drive.kSteerP,
      Constants.drive.kSteerI,
      Constants.drive.kSteerD,
      new TrapezoidProfile.Constraints(altSpeed * 2 * Math.PI, altAcc * 2 * Math.PI));
    }
    if (encoderOffset == Constants.drive.kSteerOffsetBackRight) {
      m_turningPIDController = new ProfiledPIDController(
      Constants.drive.kSteerP,
      Constants.drive.kSteerI,
      Constants.drive.kSteerD,
      new TrapezoidProfile.Constraints(altSpeed * 2 * Math.PI, Constants.drive.kMaxAngularAccel));
    }

    m_driveMotor = new WPI_TalonFX(driveMotorPort, Constants.kCanivoreCAN);
    m_steerMotor = new WPI_TalonFX(steerMotorPort, Constants.kCanivoreCAN);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_steerMotor.setNeutralMode(NeutralMode.Brake);

    m_driveEncoder = new TalonEncoder(m_driveMotor);
    m_encoder = new WPI_CANCoder(encoderPort, Constants.kCanivoreCAN);

    // reset encoder to factory defaults, reset position to the measurement of the
    // absolute encoder
    // by default the CANcoder sets it's feedback coefficient to 0.087890625, to
    // make degrees.
    m_encoder.configFactoryDefault();
    m_encoder.setPositionToAbsolute();

    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    m_encoder.configFeedbackCoefficient(2 * Math.PI / Constants.kCANcoderResolution, "rad", SensorTimeBase.PerSecond);

    m_offset = encoderOffset;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(
        2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kDriveGearRatio / Constants.kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous. Factor in the offset amount.
    m_turningPIDController.enableContinuousInput(-Math.PI + m_offset, Math.PI + m_offset);

    m_steerMotor.setInverted(true);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(getAngle()));
  }

  public ProfiledPIDController getSteerPID() {
    return m_turningPIDController;
  }

  public double turnFeedforward = 0.0;
  public double turnOutput = 0.0;

  public boolean isTurning = false;
  private double time = 0;
  private ArrayList<Double> times = new ArrayList<Double>();

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()));

    // Calculate the drive output from the drive PID controller.
    driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    turnOutput = m_turningPIDController.calculate(getAngle(), desiredState.angle.getRadians());

    turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    // m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    if (m_encoder.getVelocity() > 0.01 && !isTurning) {
      isTurning = true;
      time = Timer.getFPGATimestamp();
    } else if (m_encoder.getVelocity() <= 0.01 && isTurning) {
      isTurning = false;
      if (Timer.getFPGATimestamp() - time < 1) {
        times.add(Timer.getFPGATimestamp() - time);
      }
    }
    m_steerMotor.setVoltage(turnOutput + turnFeedforward); // * Constants.kMaxVoltage / RobotController.getBatteryVoltage()
  }

  public double getAngle() {
    return m_encoder.getAbsolutePosition() - m_offset;
  }

  public double getAverageTurnTime() {

    int highest = 0;
    for (int i = 1; i < times.size(); i++) {
      if (times.get(highest) < times.get(i)) {
        highest = i;
      }
    }

    int lowest = 0;
    for (int i = 1; i < times.size(); i++) {
      if (times.get(lowest) > times.get(i)) {
        lowest = i;
      }
    }

    double sum = 0;
    for (int i = 0; i < times.size(); i++) {
      if (i != highest && i != lowest) {
        sum += times.get(i);
      }
    }
    return sum / (times.size() - 2);
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getRate();
  }

  public PIDController getDrivePID() {
    return m_drivePIDController;
  }

  public void stop() {
    m_driveMotor.set(0);
    m_steerMotor.set(0);
  }
}