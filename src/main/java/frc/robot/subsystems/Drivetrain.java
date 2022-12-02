package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase implements Loggable{

  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2);

  public SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

  @Log
  public final SwerveModule m_frontLeft = new SwerveModule(
    Constants.drive.kDriveFrontLeft,
    Constants.drive.kSteerFrontLeft,
    Constants.drive.kEncoderFrontLeft,
    Constants.drive.kSteerOffsetFrontLeft
  );
  @Log
  public final SwerveModule m_frontRight = new SwerveModule(
    Constants.drive.kDriveFrontRight,
    Constants.drive.kSteerFrontRight,
    Constants.drive.kEncoderFrontRight,
    Constants.drive.kSteerOffsetFrontRight
  );
  @Log
  public final SwerveModule m_backLeft = new SwerveModule(
    Constants.drive.kDriveBackLeft,
    Constants.drive.kSteerBackLeft,
    Constants.drive.kEncoderBackLeft,
    Constants.drive.kSteerOffsetBackLeft
  );
  @Log
  public final SwerveModule m_backRight = new SwerveModule(
    Constants.drive.kDriveBackRight,
    Constants.drive.kSteerBackRight,
    Constants.drive.kEncoderBackRight,
    Constants.drive.kSteerOffsetBackRight
  );

  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(Constants.drive.kPigeon, Constants.kCanivoreCAN);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry;

  public Drivetrain() {
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon.getRotation2d());
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
    swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.drive.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
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

}