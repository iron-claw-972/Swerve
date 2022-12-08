package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.PracticeModeType;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2);

  public SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

  public final SwerveModule m_frontLeft = new SwerveModule(
    Constants.drive.kDriveFrontLeft,
    Constants.drive.kSteerFrontLeft,
    Constants.drive.kEncoderFrontLeft,
    Constants.drive.kSteerOffsetFrontLeft
  );
  public final SwerveModule m_frontRight = new SwerveModule(
    Constants.drive.kDriveFrontRight,
    Constants.drive.kSteerFrontRight,
    Constants.drive.kEncoderFrontRight,
    Constants.drive.kSteerOffsetFrontRight
  );
  public final SwerveModule m_backLeft = new SwerveModule(
    Constants.drive.kDriveBackLeft,
    Constants.drive.kSteerBackLeft,
    Constants.drive.kEncoderBackLeft,
    Constants.drive.kSteerOffsetBackLeft
  );
  public final SwerveModule m_backRight = new SwerveModule(
    Constants.drive.kDriveBackRight,
    Constants.drive.kSteerBackRight,
    Constants.drive.kEncoderBackRight,
    Constants.drive.kSteerOffsetBackRight
  );

  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(Constants.drive.kPigeon, Constants.kCanivoreCAN);

  public final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry;

  private PIDController xController = new PIDController(0, 0, 0);
  private PIDController yController = new PIDController(0, 0, 0);
  private PIDController rotationController = new PIDController(0.1, 0, 0);

  public Drivetrain() {
    m_odometry = new SwerveDriveOdometry(kinematics, m_pigeon.getRotation2d());
  }

  @Override
  public void periodic() {
    
    updateOdometry();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    if (Robot.shuffleboard.getPracticeModeType() == PracticeModeType.HEADING_PID_TUNE) {
      runHeadingPID();
      return;
    }

    swerveModuleStates =
        kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.drive.kMaxSpeed);
    setModuleStates(swerveModuleStates);
  }

  private void runHeadingPID() {
    double headingOutput = rotationController.calculate(getAngle(), Robot.shuffleboard.getRequestedHeading()); // should be in rad/s
      
    // headingOutput is in rad/s. Need to convert to m/s by multiplying by radius
    headingOutput *= Math.sqrt(0.5 * Constants.drive.kTrackWidth * Constants.drive.kTrackWidth);

    swerveModuleStates = new SwerveModuleState[] {
      new SwerveModuleState(headingOutput, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(headingOutput, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(headingOutput, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(headingOutput, new Rotation2d(Units.degreesToRadians(45)))
    };
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_pigeon.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  /**
   * Returns the angular rate from the pigeon.
   * 
   * @param id 0 for x, 1 for y, 2 for z
   * @return the rate in rads/s from the pigeon
   */
  public double getAngularRate(int id) {
    double[] rawGyros = new double[3];
    m_pigeon.getRawGyro(rawGyros);
    return rawGyros[id] * Math.PI / 180;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose, Rotation2d gyroAngle ){
      m_odometry.resetPosition(pose, gyroAngle);
  }

  public Rotation2d getRotation2d(){
    return m_pigeon.getRotation2d(); 
  }

  /**
   * Gets the angle heading from the pigeon
   * @return the heading angle in radians, from -pi to pi
   */
  public double getAngle() {
    Rotation2d angle = m_pigeon.getRotation2d();
    return Math.atan2(angle.getSin(), angle.getCos());
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public PIDController getXController() {
      return xController;
  }
  public PIDController getYController() {
      return yController;
  }
  public PIDController getRotationController() {
    return rotationController;
  }

}