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
import frc.robot.subsystems.Drivetrain.ModuleLocation;

public class ShuffleboardManager {

  SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drive");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");

  NetworkTableEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();
  
  public void setup() {
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns

    chooserUpdate();

    m_autoTab.add("Auto Chooser", m_autoCommand);
    setupDrivetrain();
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

  private void setupDrivetrain() {
    m_driveTab.addNumber("Angle Front Left",  () -> Robot.drive.getModuleAngle(ModuleLocation.FRONT_LEFT));
    m_driveTab.addNumber("Angle Front Right", () -> Robot.drive.getModuleAngle(ModuleLocation.FRONT_RIGHT));
    m_driveTab.addNumber("Angle Back Left",   () -> Robot.drive.getModuleAngle(ModuleLocation.BACK_LEFT));
    m_driveTab.addNumber("Angle Back Right",  () -> Robot.drive.getModuleAngle(ModuleLocation.BACK_RIGHT));

    m_driveTab.add("FL Steer PID", Robot.drive.m_frontLeft.getSteerPID());
    m_driveTab.add("FR Steer PID", Robot.drive.m_frontRight.getSteerPID());
    m_driveTab.add("BL Steer PID", Robot.drive.m_backLeft.getSteerPID());
    m_driveTab.add("BR Steer PID", Robot.drive.m_backRight.getSteerPID());
  }

}
