package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.ModuleIO;

public class ShuffleboardManager {

    SendableChooser<Command> m_autoCommand = new SendableChooser<>();

    ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
    ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
    ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drive");

    NetworkTableEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();
    
    public void setup() {
        LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns

        chooserUpdate();

        m_autoTab.add("Auto Chooser", m_autoCommand);

        int i = 0;
        for (ModuleIO module : Robot.drive.moduleIOs) {
            i++;
            m_driveTab.add("Module " + i + " Drive PID", module.getDrivePIDController());
            m_driveTab.add("Module " + i + " Steer PID", module.getSteerPIDController());

            m_driveTab.add("Module " + i + " Drive FF", module.getDriveFF());
            m_driveTab.add("Module " + i + " Steer FF", module.getSteerFF());
        }
    }

    public Command getAutonomousCommand() {
        return m_autoCommand.getSelected();
    }

    public void chooserUpdate() {
        m_autoCommand.addOption("Do Nothing", new PrintCommand("This will do nothing!"));
    }

    public void loadCommandSchedulerShuffleboard(){
        CommandScheduler.getInstance().onCommandInitialize(command -> m_commandScheduler.setString(command.getName() + " initialized."));
        CommandScheduler.getInstance().onCommandInterrupt(command -> m_commandScheduler.setString(command.getName() + " interrupted."));
        CommandScheduler.getInstance().onCommandFinish(command -> m_commandScheduler.setString(command.getName() + " finished."));
    }

}
