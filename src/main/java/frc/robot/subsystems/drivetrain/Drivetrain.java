package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.DrivetrainIO.DrivetrainIOInputs;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    // private final Translation2d m_frontLeftLocation = new Translation2d(Constants.drive.KTrackWidth / 2, Constants.drive.KTrackWidth / 2);
    // private final Translation2d m_frontRightLocation = new Translation2d(Constants.drive.KTrackWidth / 2, -Constants.drive.KTrackWidth / 2);
    // private final Translation2d m_backLeftLocation = new Translation2d(-Constants.drive.KTrackWidth / 2, Constants.drive.KTrackWidth / 2);
    // private final Translation2d m_backRightLocation = new Translation2d(-Constants.drive.KTrackWidth / 2, -Constants.drive.KTrackWidth / 2);

    // private final SwerveModule m_frontLeft = new SwerveModule(
    //     Constants.drive.FRONT_LEFT_MODULE_DRIVE_MOTOR,
    //     Constants.drive.FRONT_LEFT_MODULE_STEER_MOTOR,
    //     Constants.drive.FRONT_LEFT_MODULE_STEER_ENCODER
    // );

    // private final SwerveModule m_frontRight = new SwerveModule(
    //     Constants.drive.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
    //     Constants.drive.FRONT_RIGHT_MODULE_STEER_MOTOR,
    //     Constants.drive.FRONT_RIGHT_MODULE_STEER_ENCODER
    // );
    
    // private final SwerveModule m_backLeft = new SwerveModule(
    //       Constants.drive.BACK_LEFT_MODULE_DRIVE_MOTOR,
    //       Constants.drive.BACK_LEFT_MODULE_STEER_MOTOR,
    //       Constants.drive.BACK_LEFT_MODULE_STEER_ENCODER
    // );
    // private final SwerveModule m_backRight = new SwerveModule(
    //   Constants.drive.BACK_RIGHT_MODULE_DRIVE_MOTOR,
    //   Constants.drive.BACK_RIGHT_MODULE_STEER_MOTOR,
    //   Constants.drive.BACK_RIGHT_MODULE_STEER_ENCODER
    // );

    // private final AHRS m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    // private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    //         m_frontLeftLocation, m_frontRightLocation /*, m_backLeftLocation, m_backRightLocation*/
    // );

    // private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_navX.getRotation2d());

    private final DrivetrainIO m_drivetrainIO;

    private final DrivetrainIOInputs m_inputs = new DrivetrainIOInputs();

    public Drivetrain(DrivetrainIO io) {
        // m_navX.reset();
        m_drivetrainIO = io;
    }

    @Override
    public void periodic() {
        m_drivetrainIO.updateInputs(m_inputs);
        Logger.getInstance().processInputs("Drivetrain", m_inputs);
    }

    @Override
    public void simulationPeriodic() {
        m_drivetrainIO.simulatePeriodic();
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
        m_drivetrainIO.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_drivetrainIO.updateOdometry();
    }
}