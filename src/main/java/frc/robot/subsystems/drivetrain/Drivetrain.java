package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.BaselineConstants;

import frc.robot.subsystems.drivetrain.ModuleIO.ModuleIOInputs;
import edu.wpi.first.wpilibj.DriverStation;

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

        private final Translation2d m_frontLeftLocation = new Translation2d(Constants.drive.kTrackWidth / 2,
                        Constants.drive.kTrackWidth / 2);
        private final Translation2d m_frontRightLocation = new Translation2d(Constants.drive.kTrackWidth / 2,
                        -Constants.drive.kTrackWidth / 2);
        private final Translation2d m_backLeftLocation = new Translation2d(-Constants.drive.kTrackWidth / 2,
                        Constants.drive.kTrackWidth / 2);
        private final Translation2d m_backRightLocation = new Translation2d(-Constants.drive.kTrackWidth / 2,
                        -Constants.drive.kTrackWidth / 2);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        m_frontLeftLocation,
                        m_frontRightLocation,
                        m_backLeftLocation,
                        m_backRightLocation);

        private Pose2d m_robotPosition = new Pose2d(0, 0, new Rotation2d());
        private final SwerveDriveOdometry m_odometry;

        public Drivetrain() {

                m_odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon.getRotation2d(), m_robotPosition);

                if (Robot.isReal) {
                        moduleIOs[0] = new ModuleIOTalon(
                                        Constants.drive.kDriveFrontLeft,
                                        Constants.drive.kSteerFrontLeft,
                                        Constants.drive.kEncoderFrontLeft,
                                        Constants.drive.kSteerOffsetFrontLeft);
                        moduleIOs[1] = new ModuleIOTalon(
                                        Constants.drive.kDriveFrontRight,
                                        Constants.drive.kSteerFrontRight,
                                        Constants.drive.kEncoderFrontRight,
                                        Constants.drive.kSteerOffsetFrontRight);
                        moduleIOs[2] = new ModuleIOTalon(
                                        Constants.drive.kDriveBackLeft,
                                        Constants.drive.kSteerBackLeft,
                                        Constants.drive.kEncoderBackLeft,
                                        Constants.drive.kSteerOffsetBackLeft);
                        moduleIOs[3] = new ModuleIOTalon(
                                        Constants.drive.kDriveBackRight,
                                        Constants.drive.kSteerBackRight,
                                        Constants.drive.kEncoderBackRight,
                                        Constants.drive.kSteerOffsetBackRight);
                } else {
                        moduleIOs[0] = new ModuleIOSim(
                                        Constants.drive.kDriveFrontLeft,
                                        Constants.drive.kSteerFrontLeft,
                                        Constants.drive.kEncoderFrontLeft,
                                        Constants.drive.kSteerOffsetFrontLeft);
                        moduleIOs[1] = new ModuleIOSim(
                                        Constants.drive.kDriveFrontRight,
                                        Constants.drive.kSteerFrontRight,
                                        Constants.drive.kEncoderFrontRight,
                                        Constants.drive.kSteerOffsetFrontRight);
                        moduleIOs[2] = new ModuleIOSim(
                                        Constants.drive.kDriveBackLeft,
                                        Constants.drive.kSteerBackLeft,
                                        Constants.drive.kEncoderBackLeft,
                                        Constants.drive.kSteerOffsetBackLeft);
                        moduleIOs[3] = new ModuleIOSim(
                                        Constants.drive.kDriveBackRight,
                                        Constants.drive.kSteerBackRight,
                                        Constants.drive.kEncoderBackRight,
                                        Constants.drive.kSteerOffsetBackRight);
                }
        }

        @Override
        public void periodic() {

                for (int i = 0; i < moduleIOs.length; i++) {
                        moduleIOs[i].updateInputs(moduleInputs[i]);
                        Logger.getInstance().processInputs("RealOutputs/Drivetrain/SwerveModule" + (i + 1),
                                        moduleInputs[i]);
                        if (moduleInputs[i].driveVelocity > BaselineConstants.speed) {
                                DriverStation.reportWarning("Baseline SPEED exceeded! Current SPEED: "
                                                + moduleInputs[i].driveVelocity + " and baseline is "
                                                + BaselineConstants.speed, false);
                        }

                        if (moduleInputs[i].driveAppliedVolts > BaselineConstants.driveVoltage) {
                                DriverStation.reportWarning(
                                                "Baseline DRIVE VOLTAGE exceeded! Current DRIVE VOLTAGE: "
                                                                + moduleInputs[i].driveAppliedVolts
                                                                + " and baseline is " + BaselineConstants.driveVoltage,
                                                false);
                        }

                        if (moduleInputs[i].steerAppliedVolts > BaselineConstants.steerVoltage) {
                                DriverStation.reportWarning(
                                                "Baseline STEER VOLTAGE exceeded! Current STEER VOLTAGE: "
                                                                + moduleInputs[i].steerAppliedVolts
                                                                + " and baseline is " + BaselineConstants.steerVoltage,
                                                false);
                        }

                        if (moduleInputs[i].driveCurrentAmps[0] > BaselineConstants.driveCurrentAmps) {
                                DriverStation.reportWarning(
                                                "Baseline DRIVE CURRENT AMPS exceeded! Current DRIVE CURRENT AMPS: "
                                                                + moduleInputs[i].driveCurrentAmps[0]
                                                                + " and baseline is "
                                                                + BaselineConstants.driveCurrentAmps,
                                                false);
                        }

                        if (moduleInputs[i].steerCurrentAmps[0] > BaselineConstants.steerCurrentAmps) {
                                DriverStation.reportWarning(
                                                "Baseline STEER CURRENT AMPS exceeded! Current STEER CURRENT AMPS: "
                                                                + moduleInputs[i].steerCurrentAmps[0]
                                                                + " and baseline is "
                                                                + BaselineConstants.steerCurrentAmps,
                                                false);
                        }

                        Logger.getInstance().processInputs("RealOutputs/Drivetrain/SwerveModule" + (i + 1),
                                        moduleInputs[i]);
                }

                Logger.getInstance().recordOutput("Drivetrain/Gyro/Angle", m_pigeon.getAngle());
                Logger.getInstance().recordOutput("Drivetrain/Gyro/AngularVelocity", m_pigeon.getRate());

                Logger.getInstance().recordOutput("Drivetrain/RobotPose", m_robotPosition);

                updateOdometry();
        }

        public void updateOdometry() {
                if (/* Robot.isReal */true) {
                        m_robotPosition = m_odometry.update(
                                        m_pigeon.getRotation2d(),
                                        m_modulePositions);
                }
                // TODO: Deal with this code or something
                /*
                 * else {
                 * //FIXME: figure out how to make this work, from advantage kit
                 * ChassisSpeeds chassisState = m_kinematics.toChassisSpeeds(
                 * moduleIOs[0].getState(), moduleIOs[1].getState(), moduleIOs[2].getState(),
                 * moduleIOs[3].getState()
                 * );
                 * 
                 * m_robotPosition = m_robotPosition.exp(new Twist2d(
                 * chassisState.vxMetersPerSecond,
                 * chassisState.vyMetersPerSecond,
                 * chassisState.omegaRadiansPerSecond)
                 * );
                 * }
                 */
        }

        /**
         * @param fieldRelative Whether the provided x and y speeds are relative to the
         *                      field.
         */
        public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
                // TODO: This could be potentially replaced by a better solution to making the
                // robot turn in sim
                if (!Robot.isReal)
                        m_pigeon.getSimCollection().addHeading(rot / (2 * Math.PI));

                var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                                fieldRelative
                                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                                                m_pigeon.getRotation2d())
                                                : new ChassisSpeeds(xSpeed, ySpeed, rot));
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.drive.kMaxSpeed);
                for (int i = 0; i < moduleIOs.length; i++) {
                        moduleIOs[i].setDesiredState(swerveModuleStates[i]);
                }
        }

        public Pose2d getRobotPosition() {
                return m_robotPosition;
        }

        public Pose2d getRobotPosition() {
                return m_robotPosition;
        }
}