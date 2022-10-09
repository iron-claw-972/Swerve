package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.constants.Constants;

public class DrivetrainIOSim implements DrivetrainIO {

    private final Translation2d m_frontLeftLocation = new Translation2d(Constants.drive.KTrackWidth / 2, Constants.drive.KTrackWidth / 2);
    private final Translation2d m_frontRightLocation = new Translation2d(Constants.drive.KTrackWidth / 2, -Constants.drive.KTrackWidth / 2);
    private final Translation2d m_backLeftLocation = new Translation2d(-Constants.drive.KTrackWidth / 2, Constants.drive.KTrackWidth / 2);
    private final Translation2d m_backRightLocation = new Translation2d(-Constants.drive.KTrackWidth / 2, -Constants.drive.KTrackWidth / 2);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation , m_backLeftLocation, m_backRightLocation
    );

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

    public DrivetrainIOSim() {
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        inputs.frontLeft_driveVelocity = m_frontLeft.getDriveVelocity();
        inputs.frontLeft_steerAngle = m_frontLeft.getSteerAngle();
    }

    @Override
    public void simulatePeriodic() {

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
        m_navX.setAngleAdjustment(m_angle);
        
    }
    
    
}
