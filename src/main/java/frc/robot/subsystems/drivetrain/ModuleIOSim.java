package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;

import ctre_shims.TalonEncoder;
import ctre_shims.TalonEncoderSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.Constants;

public class ModuleIOSim implements ModuleIO {

    // TODO: need these values!
    private FlywheelSim m_driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025);
    private FlywheelSim m_steerMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004096955);

    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonFX m_steerMotor;

    // private final TalonFXSimCollection m_driveMotorSim;
    // private final TalonFXSimCollection m_steerMotorSim;

    private final TalonEncoder m_driveEncoder;
    private final CANCoder m_encoder;

    private final TalonEncoderSim m_driveEncoderSim;
    private final CANCoderSimCollection m_encoderSim;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
        1, 
        0, 
        0,
        new TrapezoidProfile.Constraints(Constants.swerve.kMaxAngularSpeed, 2 * Math.PI)
        );

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 0);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0);

    private double m_currentTurnPositionRad = 0;
    private double m_absoluteTurnPositionRad = 0;

    public ModuleIOSim(
        int driveMotorPort,
        int steerMotorPort,
        int encoderPort) {
        m_driveMotor = new WPI_TalonFX(driveMotorPort);
        m_steerMotor = new WPI_TalonFX(steerMotorPort);

        // m_driveMotorSim = m_driveMotor.getSimCollection();
        // m_steerMotorSim = m_steerMotor.getSimCollection();

        m_driveEncoder = new TalonEncoder(m_driveMotor);
        m_encoder = new CANCoder(encoderPort);

        m_driveEncoderSim = new TalonEncoderSim(m_driveEncoder);
        m_encoderSim = new CANCoderSimCollection(m_encoder);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveEncoder.setDistancePerPulse(2 * Math.PI * Constants.swerve.kWheelRadius / 6.75 / Constants.kEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        m_driveMotorSim.update(0.02);
        m_steerMotorSim.update(0.02);

        inputs.driveVelocity = m_driveEncoderSim.getRate();
        inputs.driveAppliedVolts = 0;
        inputs.driveCurrentAmps = new double[] {m_driveMotorSim.getCurrentDrawAmps()};
        inputs.driveTempCelcius = new double[] {};

        inputs.steerAngle = new Rotation2d(m_steerMotor.get()).getDegrees();
        inputs.steerAppliedVolts = 0;
        inputs.steerCurrentAmps = new double[] {m_steerMotorSim.getCurrentDrawAmps()};
        inputs.steerTempCelcius = new double[] {};

        double angleDiffRad = m_steerMotorSim.getAngularVelocityRadPerSec() * 0.02;
        m_currentTurnPositionRad += angleDiffRad;
        m_absoluteTurnPositionRad += angleDiffRad;
        while (m_absoluteTurnPositionRad < 0) {
            m_absoluteTurnPositionRad += 2 * Math.PI;
        }
        while (m_absoluteTurnPositionRad > 2 * Math.PI) {
            m_absoluteTurnPositionRad -= 2 * Math.PI;
        }
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveMotorSim.getAngularVelocityRPM() * Constants.swerve.kWheelRadius * 2 * Math.PI / 60, new Rotation2d(m_currentTurnPositionRad));
        // return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(m_steerMotor.get()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(m_currentTurnPositionRad));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
            m_drivePIDController.calculate(m_driveEncoder.getRate(), desiredState.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        
        final double turnOutput = m_turningPIDController.calculate(m_currentTurnPositionRad, desiredState.angle.getRadians());

        final double turnFeedforward =
            m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotorSim.setInputVoltage(driveOutput * 5 + driveFeedforward);
        m_steerMotorSim.setInputVoltage(turnOutput + turnFeedforward);
    }
    
}
