package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Driver;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DefaultDriveCommand extends CommandBase {
    
    private final Drivetrain m_drive;

    public DefaultDriveCommand(Drivetrain drive) {
        m_drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double xSpeed = Driver.getForwardTranslation();
        double ySpeed = Driver.getSideTranslation();
        double rot = Driver.getRotation();

        Logger.getInstance().recordOutput("DefaultDriveCommand/xSpeed", xSpeed);
        Logger.getInstance().recordOutput("DefaultDriveCommand/ySpeed", ySpeed);
        Logger.getInstance().recordOutput("DefaultDriveCommand/rotation", rot);
    
        m_drive.drive(xSpeed, ySpeed, rot, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0.0, 0.0, 0.0, false);
    }
}
