package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DrivetrainIO.DrivetrainIOInputs;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

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
        updateOdometry();
    }

    @Override
    public void simulationPeriodic() {
        m_drivetrainIO.simulationPeriodic();
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

    public DrivetrainIO getDrivetrainIO() {
        return m_drivetrainIO;
    }
}