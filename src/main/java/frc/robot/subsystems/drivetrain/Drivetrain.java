package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.ModuleIO.ModuleIOInputs;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

    private WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(Constants.drive.kPigeon, Constants.kCanivoreCAN);

    public final ModuleIO[] moduleIOs = new ModuleIO[4];
    public ModuleIO[] getModuleIOs() {
        return moduleIOs;
    }

    private final ModuleIOInputs[] moduleInputs = new ModuleIOInputs[] {
        new ModuleIOInputs(),
        new ModuleIOInputs(),
        new ModuleIOInputs(),
        new ModuleIOInputs()
    };


    private final Translation2d m_frontLeftLocation = new Translation2d(Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2);
    private final Translation2d m_frontRightLocation = new Translation2d(Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2);
    private final Translation2d m_backLeftLocation = new Translation2d(-Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2);
    private final Translation2d m_backRightLocation = new Translation2d(-Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation,
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation
    );

    private Pose2d m_robotPosition = new Pose2d(0, 0, new Rotation2d());
    private final SwerveDriveOdometry m_odometry;

    public Drivetrain() {

        m_odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon.getRotation2d(), m_robotPosition);

        if (Robot.isReal) {
            moduleIOs[0] = new ModuleIOTalon(
                Constants.drive.kDriveFrontLeft,
                Constants.drive.kSteerFrontLeft,
                Constants.drive.kEncoderFrontLeft,
                Constants.drive.kSteerOffsetFrontLeft
            );
            moduleIOs[1] = new ModuleIOTalon(
                Constants.drive.kDriveFrontRight,
                Constants.drive.kSteerFrontRight,
                Constants.drive.kEncoderFrontRight,
                Constants.drive.kSteerOffsetFrontRight
            );
            moduleIOs[2] = new ModuleIOTalon(
                Constants.drive.kDriveBackLeft,
                Constants.drive.kSteerBackLeft,
                Constants.drive.kEncoderBackLeft,
                Constants.drive.kSteerOffsetBackLeft
            );
            moduleIOs[3] = new ModuleIOTalon(
                Constants.drive.kDriveBackRight,
                Constants.drive.kSteerBackRight,
                Constants.drive.kEncoderBackRight,
                Constants.drive.kSteerOffsetBackRight
            );
        } else {
            moduleIOs[0] = new ModuleIOSim(
                Constants.drive.kDriveFrontLeft,
                Constants.drive.kSteerFrontLeft,
                Constants.drive.kEncoderFrontLeft,
                Constants.drive.kSteerOffsetFrontLeft
            );
            moduleIOs[1] = new ModuleIOSim(
                Constants.drive.kDriveFrontRight,
                Constants.drive.kSteerFrontRight,
                Constants.drive.kEncoderFrontRight,
                Constants.drive.kSteerOffsetFrontRight
            );
            moduleIOs[2] = new ModuleIOSim(
                Constants.drive.kDriveBackLeft,
                Constants.drive.kSteerBackLeft,
                Constants.drive.kEncoderBackLeft,
                Constants.drive.kSteerOffsetBackLeft
            );
            moduleIOs[3] = new ModuleIOSim(
                Constants.drive.kDriveBackRight,
                Constants.drive.kSteerBackRight,
                Constants.drive.kEncoderBackRight,
                Constants.drive.kSteerOffsetBackRight
            );
        }
    }

    @Override
    public void periodic() {

        for (int i = 0; i < moduleIOs.length; i++) {
            moduleIOs[i].updateInputs(moduleInputs[i]);
            Logger.getInstance().processInputs("RealOutputs/Drivetrain/SwerveModule" + (i+1), moduleInputs[i]);
        }

        Logger.getInstance().recordOutput("Drivetrain/Gyro/Angle", m_pigeon.getAngle());
        Logger.getInstance().recordOutput("Drivetrain/Gyro/AngularVelocity", m_pigeon.getRate());

        double[] loggedRobotPose = new double[] {
            m_robotPosition.getTranslation().getX(),
            m_robotPosition.getTranslation().getY(),
            m_robotPosition.getRotation().getDegrees()
        };

        Logger.getInstance().recordOutput("Drivetrain/RobotPose", loggedRobotPose);

        updateOdometry();
    }

    public void updateOdometry() {
        if (/*Robot.isReal*/true) {
            m_robotPosition = m_odometry.update(
                m_pigeon.getRotation2d(), //may need to be negative or something else
                moduleIOs[0].getState(),
                moduleIOs[1].getState(),
                moduleIOs[2].getState(),
                moduleIOs[3].getState()
            );
        } else {

            //FIXME: figure out how to make this work, from advantage kit
            ChassisSpeeds chassisState = m_kinematics.toChassisSpeeds(
                moduleIOs[0].getState(), moduleIOs[1].getState(), moduleIOs[2].getState(), moduleIOs[3].getState()
            );

            m_robotPosition = m_robotPosition.exp(new Twist2d(
              chassisState.vxMetersPerSecond,
              chassisState.vyMetersPerSecond,
              chassisState.omegaRadiansPerSecond)
            );
        }
    }

    /**
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (!Robot.isReal) m_pigeon.getSimCollection().addHeading(rot / (2 * Math.PI));
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
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