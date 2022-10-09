package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.constants.Constants;

public class DrivetrainIOReal implements DrivetrainIO {

    private final Translation2d m_frontLeftLocation = new Translation2d(Constants.drive.KTrackWidth / 2, Constants.drive.KTrackWidth / 2);
    private final Translation2d m_frontRightLocation = new Translation2d(Constants.drive.KTrackWidth / 2, -Constants.drive.KTrackWidth / 2);

    private final SwerveModule m_frontLeft = new SwerveModule(
        Constants.drive.FRONT_LEFT_MODULE_DRIVE_MOTOR,
        Constants.drive.FRONT_LEFT_MODULE_STEER_MOTOR,
        Constants.drive.FRONT_LEFT_MODULE_STEER_ENCODER
    );

    private final SwerveModule m_frontRight = new SwerveModule(
        Constants.drive.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.drive.FRONT_RIGHT_MODULE_STEER_MOTOR,
        Constants.drive.FRONT_RIGHT_MODULE_STEER_ENCODER
    );

    private final SwerveModule m_backLeft = new SwerveModule(
          Constants.drive.BACK_LEFT_MODULE_DRIVE_MOTOR,
          Constants.drive.BACK_LEFT_MODULE_STEER_MOTOR,
          Constants.drive.BACK_LEFT_MODULE_STEER_ENCODER
    );
    private final SwerveModule m_backRight = new SwerveModule(
      Constants.drive.BACK_RIGHT_MODULE_DRIVE_MOTOR,
      Constants.drive.BACK_RIGHT_MODULE_STEER_MOTOR,
      Constants.drive.BACK_RIGHT_MODULE_STEER_ENCODER
    );

    private static final AHRS m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation /*, m_backLeftLocation, m_backRightLocation*/
    );

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_navX.getRotation2d());

    public DrivetrainIOReal() {
        System.out.println("Initialized DrivetrianIOReal");
        m_navX.reset();
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        inputs.frontLeft_driveVelocity = m_frontLeft.getDriveVelocity();
        inputs.frontLeft_steerAngle = m_frontLeft.getSteerAngle();
    }

    @Override
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_navX.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.drive.kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
    //     m_backLeft.setDesiredState(swerveModuleStates[2]);
    //     m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    @Override
    public void updateOdometry() {
        m_odometry.update(
            m_navX.getRotation2d(),
            m_frontLeft.getState(),
            m_frontRight.getState()/*,
            m_backLeft.getState(),
            m_backRight.getState()*/
            );
    }
    
}
