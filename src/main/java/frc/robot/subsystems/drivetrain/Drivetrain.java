package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.GyroIO.GyroIOInputs;
import frc.robot.subsystems.drivetrain.ModuleIO.ModuleIOInputs;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

    private final GyroIO m_gyroIO = new GyroIONavX();
    private final GyroIOInputs m_gyroInputs = new GyroIOInputs();

    private final ModuleIO[] moduleIOs = new ModuleIO[4];
    private final ModuleIOInputs[] moduleInputs = new ModuleIOInputs[] {
        new ModuleIOInputs(),
        new ModuleIOInputs(),
        new ModuleIOInputs(),
        new ModuleIOInputs()
    };

    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation,
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation
    );

    private Pose2d m_robotPosition = new Pose2d(0, 0, new Rotation2d());
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyroIO.getNavX().getRotation2d(), m_robotPosition);

    public Drivetrain() {
        if (RobotBase.isReal()) {
            moduleIOs[0] = new ModuleIOTalon(
                Constants.drive.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Constants.drive.FRONT_LEFT_MODULE_STEER_MOTOR,
                Constants.drive.FRONT_LEFT_MODULE_STEER_ENCODER
            );
            moduleIOs[1] = new ModuleIOTalon(
                Constants.drive.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.drive.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Constants.drive.FRONT_RIGHT_MODULE_STEER_ENCODER
            );
            moduleIOs[2] = new ModuleIOTalon(
                Constants.drive.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.drive.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.drive.BACK_LEFT_MODULE_STEER_ENCODER
            );
            moduleIOs[3] = new ModuleIOTalon(
                Constants.drive.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.drive.BACK_RIGHT_MODULE_STEER_MOTOR,
                Constants.drive.BACK_RIGHT_MODULE_STEER_ENCODER
            );
        } else {
            moduleIOs[0] = new ModuleIOSim(
                Constants.drive.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Constants.drive.FRONT_LEFT_MODULE_STEER_MOTOR,
                Constants.drive.FRONT_LEFT_MODULE_STEER_ENCODER
            );
            moduleIOs[1] = new ModuleIOSim(
                Constants.drive.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.drive.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Constants.drive.FRONT_RIGHT_MODULE_STEER_ENCODER
            );
            moduleIOs[2] = new ModuleIOSim(
                Constants.drive.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.drive.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.drive.BACK_LEFT_MODULE_STEER_ENCODER
            );
            moduleIOs[3] = new ModuleIOSim(
                Constants.drive.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.drive.BACK_RIGHT_MODULE_STEER_MOTOR,
                Constants.drive.BACK_RIGHT_MODULE_STEER_ENCODER
            );
        }
    }

    @Override
    public void periodic() {
        m_gyroIO.updateInputs(m_gyroInputs);
        Logger.getInstance().processInputs("Drivetrain/Gyro", m_gyroInputs);

        for (int i = 0; i < moduleIOs.length; i++) {
            moduleIOs[i].updateInputs(moduleInputs[i]);
            Logger.getInstance().processInputs("Drivetrain/SwerveModule" + (i+1), moduleInputs[i]);
        }

        updateOdometry();
    }

    public void updateOdometry() {
        m_robotPosition = m_odometry.update(
            Rotation2d.fromDegrees(-m_gyroIO.getNavX().getAngle()),
            moduleIOs[0].getState(),
            moduleIOs[1].getState(),
            moduleIOs[2].getState(),
            moduleIOs[3].getState()
        );
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
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyroIO.getNavX().getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.drive.kMaxSpeed);
        for (int i = 0; i < moduleIOs.length; i++) {
            moduleIOs[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public Pose2d getRobotPosition() {
        return m_robotPosition;
    } 
}