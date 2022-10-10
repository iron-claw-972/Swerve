package frc.robot.subsystems.unused;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.constants.Constants;
import frc.robot.subsystems.unused.DrivetrainIO.DrivetrainIOInputs;

public class DrivetrainIOSim implements DrivetrainIO {

    private final Translation2d m_frontLeftLocation = new Translation2d(Constants.drive.KTrackWidth / 2, Constants.drive.KTrackWidth / 2);
    private final Translation2d m_frontRightLocation = new Translation2d(Constants.drive.KTrackWidth / 2, -Constants.drive.KTrackWidth / 2);
    private final Translation2d m_backLeftLocation = new Translation2d(-Constants.drive.KTrackWidth / 2, Constants.drive.KTrackWidth / 2);
    private final Translation2d m_backRightLocation = new Translation2d(-Constants.drive.KTrackWidth / 2, -Constants.drive.KTrackWidth / 2);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation , m_backLeftLocation, m_backRightLocation
    );

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_navX.getRotation2d());

    private static final AHRS m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    private final SwerveModuleSim m_frontLeft = new SwerveModuleSim(
        Constants.drive.FRONT_LEFT_MODULE_DRIVE_MOTOR,
        Constants.drive.FRONT_LEFT_MODULE_STEER_MOTOR,
        Constants.drive.FRONT_LEFT_MODULE_STEER_ENCODER
    );

    private final SwerveModuleSim m_frontRight = new SwerveModuleSim(
        Constants.drive.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.drive.FRONT_RIGHT_MODULE_STEER_MOTOR,
        Constants.drive.FRONT_RIGHT_MODULE_STEER_ENCODER
    );

    private final SwerveModuleSim m_backLeft = new SwerveModuleSim(
          Constants.drive.BACK_LEFT_MODULE_DRIVE_MOTOR,
          Constants.drive.BACK_LEFT_MODULE_STEER_MOTOR,
          Constants.drive.BACK_LEFT_MODULE_STEER_ENCODER
    );

    private final SwerveModuleSim m_backRight = new SwerveModuleSim(
          Constants.drive.BACK_RIGHT_MODULE_DRIVE_MOTOR,
          Constants.drive.BACK_RIGHT_MODULE_STEER_MOTOR,
          Constants.drive.BACK_RIGHT_MODULE_STEER_ENCODER
    );

    private double m_angle = 0;
    int m_navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

    public DrivetrainIOSim() {
    }

    private Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(-m_navX.getAngle(), 360));
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        inputs.frontLeft_driveVelocity = m_frontLeft.getDriveVelocity();
        inputs.frontLeft_steerAngle = m_frontLeft.getSteerAngle();

        inputs.frontRight_driveVelocity = m_frontRight.getDriveVelocity();
        inputs.frontRight_steerAngle = m_frontRight.getSteerAngle();

        inputs.backLeft_driveVelocity = m_backLeft.getDriveVelocity();
        inputs.backLeft_steerAngle = m_backLeft.getSteerAngle();

        inputs.backRight_driveVelocity = m_backRight.getDriveVelocity();
        inputs.backRight_steerAngle = m_backRight.getSteerAngle();

        Logger.getInstance().recordOutput("LocationX", m_odometry.getPoseMeters().getTranslation().getX());
        Logger.getInstance().recordOutput("LocationY", m_odometry.getPoseMeters().getTranslation().getY());
    }

    @Override
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.drive.kMaxSpeed);

        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    @Override
    public void updateOdometry() {
        m_odometry.update(
            getRotation2d(),
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
            );
    }

    @Override
    public Pose2d getOdometry() {
        return m_odometry.getPoseMeters();
    }

    @Override
    public void simulationPeriodic() {

        m_frontLeft.simulationPeriodic(0.02);
        m_frontRight.simulationPeriodic(0.02);
        m_backLeft.simulationPeriodic(0.02);
        m_backRight.simulationPeriodic(0.02);

        SwerveModuleState[] moduleStates = {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };

        var chassisSpeeds = m_kinematics.toChassisSpeeds(moduleStates);
        double chassisRotationSpeed = chassisSpeeds.omegaRadiansPerSecond;

        m_angle += chassisRotationSpeed * 0.02;
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_navXSim, "Yaw"));
        angle.set(m_angle);        
    }
    
    
}
