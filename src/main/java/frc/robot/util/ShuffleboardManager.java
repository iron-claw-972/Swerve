package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Robot;
import frc.robot.commands.auto.PathPlannerCommand;

public class ShuffleboardManager {

  SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  public ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drive");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");

  NetworkTableEntry m_heading = m_driveTab.add("Set Heading (-pi to pi)", 0).getEntry();

  NetworkTableEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();

  SendableChooser<PracticeModeType> m_practiceMode = new SendableChooser<>();
  
  public void setup() {
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns

    autoChooserUpdate();
    practiceChooserUpdate();

    m_autoTab.add("Auto Chooser", m_autoCommand);
    m_mainTab.add("Practice Mode Type Chooser", m_practiceMode);

    setupDrivetrain();

    m_driveTab.add("xController", Robot.drive.getXController());
    m_driveTab.add("yController", Robot.drive.getYController());
    m_driveTab.add("rotationController", Robot.drive.getRotationController());
    m_driveTab.addNumber("getAngle", () -> Robot.drive.getAngle());
  }

  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }

  public void autoChooserUpdate() {
    m_autoCommand.addOption("Do Nothing", new PrintCommand("This will do nothing!"));
    m_autoCommand.setDefaultOption("TestAuto", new PathPlannerCommand("TestAuto", 0)); 
  }

  public PracticeModeType getPracticeModeType() {
    return m_practiceMode.getSelected();
  }

  public void practiceChooserUpdate() {
    m_practiceMode.addOption(PracticeModeType.HEADING_PID_TUNE.toString(), PracticeModeType.HEADING_PID_TUNE);
    m_practiceMode.setDefaultOption(PracticeModeType.NONE.toString(), PracticeModeType.NONE);
  }

  public double getRequestedHeading() {
    return m_heading.getDouble(0);
  }

  public void loadCommandSchedulerShuffleboard(){
    CommandScheduler.getInstance().onCommandInitialize(command -> m_commandScheduler.setString(command.getName() + " initialized."));
    CommandScheduler.getInstance().onCommandInterrupt(command -> m_commandScheduler.setString(command.getName() + " interrupted."));
    CommandScheduler.getInstance().onCommandFinish(command -> m_commandScheduler.setString(command.getName() + " finished."));
  }

  private void setupDrivetrain() {
    m_driveTab.addNumber("Angle Front Left",  () -> Robot.drive.m_frontLeft.getAngle());
    m_driveTab.addNumber("Angle Front Right", () -> Robot.drive.m_frontRight.getAngle());
    m_driveTab.addNumber("Angle Back Left",   () -> Robot.drive.m_backLeft.getAngle());
    m_driveTab.addNumber("Angle Back Right",  () -> Robot.drive.m_backRight.getAngle());

    m_driveTab.addNumber("Gyro X", () -> Robot.drive.getAngularRate(0));
    m_driveTab.addNumber("Gyro Y", () -> Robot.drive.getAngularRate(1));
    m_driveTab.addNumber("Gyro Z", () -> Robot.drive.getAngularRate(2));

    // m_driveTab.addNumber("FL desired speed", () -> Robot.drive.swerveModuleStates[0].speedMetersPerSecond);
    // m_driveTab.addNumber("FR desired speed", () -> Robot.drive.swerveModuleStates[1].speedMetersPerSecond);
    // m_driveTab.addNumber("BL desired speed", () -> Robot.drive.swerveModuleStates[2].speedMetersPerSecond);
    // m_driveTab.addNumber("BR desired speed", () -> Robot.drive.swerveModuleStates[3].speedMetersPerSecond);

    // m_driveTab.addNumber("FL desired angle", () -> Robot.drive.swerveModuleStates[0].angle.getDegrees());
    // m_driveTab.addNumber("FR desired angle", () -> Robot.drive.swerveModuleStates[1].angle.getDegrees());
    // m_driveTab.addNumber("BL desired angle", () -> Robot.drive.swerveModuleStates[2].angle.getDegrees());
    // m_driveTab.addNumber("BR desired angle", () -> Robot.drive.swerveModuleStates[3].angle.getDegrees());

    m_driveTab.addNumber("FL FF", () -> Robot.drive.m_frontLeft.turnFeedforward);
    m_driveTab.addNumber("FR FF", () -> Robot.drive.m_frontRight.turnFeedforward);
    m_driveTab.addNumber("BL FF", () -> Robot.drive.m_backLeft.turnFeedforward);
    m_driveTab.addNumber("BR FF", () -> Robot.drive.m_backRight.turnFeedforward);

    m_driveTab.addNumber("FL PID Output", () -> Robot.drive.m_frontLeft.turnOutput);
    m_driveTab.addNumber("FR PID Output", () -> Robot.drive.m_frontRight.turnOutput);
    m_driveTab.addNumber("BL PID Output", () -> Robot.drive.m_backLeft.turnOutput);
    m_driveTab.addNumber("BR PID Output", () -> Robot.drive.m_backRight.turnOutput);


    // m_driveTab.addNumber("Vel Front Right", () -> Robot.drive.m_frontRight.getDriveVelocity());
    // m_driveTab.addNumber("Drive Output Front Right", () -> Robot.drive.m_frontRight.driveOutput);
    // m_driveTab.add("FR Drive PID", Robot.drive.m_frontRight.getDrivePID());

    // m_driveTab.addNumber("Vel Front Left", () -> Robot.drive.m_frontLeft.getDriveVelocity());
    // m_driveTab.addNumber("Drive Output Front Left", () -> Robot.drive.m_frontLeft.driveOutput);
    // m_driveTab.add("FL Drive PID", Robot.drive.m_frontLeft.getDrivePID());
  }

}
