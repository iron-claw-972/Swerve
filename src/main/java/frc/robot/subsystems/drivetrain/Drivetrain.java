package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DrivetrainIO.DrivetrainIOInputs;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

    private DrivetrainIOReal m_driveIO = new DrivetrainIOReal();
    private DrivetrainIOInputs m_driveIOInputs = new DrivetrainIOInputs();


    public Drivetrain() {
        
    }

    @Override
    public void periodic() {
        m_driveIO.updateInputs(m_driveIOInputs);
        Logger.getInstance().processInputs("Drivetrain/Drivetrain", m_driveIOInputs);
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
        m_driveIO.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
}