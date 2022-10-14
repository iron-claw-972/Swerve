package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.GyroIO.GyroIOInputs;
import frc.robot.subsystems.drivetrain.ModuleIO.ModuleIOInputs;

public class DrivetrainIOReal implements DrivetrainIO {

    private final GyroIO m_gyroIO = new GyroIONavX();
    private final GyroIOInputs m_gyroInputs = new GyroIOInputs();

    private final ModuleIO[] moduleIOs = new ModuleIO[2];
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

    private Pose2d m_robotPosition = new Pose2d();
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyroIO.getNavX().getRotation2d(), m_robotPosition);

    public DrivetrainIOReal() {
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
            moduleIOs[0] = new ModuleIOTalon(
                Constants.drive.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.drive.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.drive.BACK_LEFT_MODULE_STEER_ENCODER
            );
            moduleIOs[1] = new ModuleIOTalon(
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
    
    
    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        
        m_robotPosition = m_odometry.update(
            Rotation2d.fromDegrees(-m_gyroIO.getNavX().getAngle()),
            moduleIOs[0].getState(),
            moduleIOs[1].getState()/*,
            moduleIOs[2].getState(),
            moduleIOs[3].getState()*/
        );

        inputs.robotPose = new double[] {m_robotPosition.getX(), m_robotPosition.getY(), m_robotPosition.getRotation().getDegrees()};

        m_gyroIO.updateInputs(m_gyroInputs);
        Logger.getInstance().processInputs("Drivetrain/Gyro", m_gyroInputs);

        for (int i = 0; i < moduleIOs.length; i++) {
            moduleIOs[i].updateInputs(moduleInputs[i]);
            Logger.getInstance().processInputs("Drivetrain/SwerveModule" + (i+1), moduleInputs[i]);
        }
    }

}
