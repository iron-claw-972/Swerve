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
  ShuffleboardTab m_swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");

  NetworkTableEntry m_heading = m_driveTab.add("Set Heading (-pi to pi)", 0).getEntry();
  NetworkTableEntry m_velocity = m_swerveModulesTab.add("Set Drive Velocity", 0).getEntry();

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
    m_driveTab.addNumber("heading PID output", () -> Robot.drive.headingPIDOutput);
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
    m_practiceMode.addOption(PracticeModeType.TUNE_HEADING_PID.toString(), PracticeModeType.TUNE_HEADING_PID);
    m_practiceMode.addOption(PracticeModeType.TUNE_MODULE_DRIVE.toString(), PracticeModeType.TUNE_MODULE_DRIVE);
    m_practiceMode.setDefaultOption(PracticeModeType.NONE.toString(), PracticeModeType.NONE);
  }

  public double getRequestedHeading() {
    return m_heading.getDouble(0);
  }

  public double getRequestedVelocity() {
    return m_velocity.getDouble(0);
  }

  public void loadCommandSchedulerShuffleboard(){
    CommandScheduler.getInstance().onCommandInitialize(command -> m_commandScheduler.setString(command.getName() + " initialized."));
    CommandScheduler.getInstance().onCommandInterrupt(command -> m_commandScheduler.setString(command.getName() + " interrupted."));
    CommandScheduler.getInstance().onCommandFinish(command -> m_commandScheduler.setString(command.getName() + " finished."));
  }

  private void setupDrivetrain() {
    m_swerveModulesTab.addNumber("Angle Front Left",  () -> Robot.drive.m_frontLeft.getAngle());
    m_swerveModulesTab.addNumber("Angle Front Right", () -> Robot.drive.m_frontRight.getAngle());
    m_swerveModulesTab.addNumber("Angle Back Left",   () -> Robot.drive.m_backLeft.getAngle());
    m_swerveModulesTab.addNumber("Angle Back Right",  () -> Robot.drive.m_backRight.getAngle());

    m_driveTab.addNumber("Gyro X", () -> Robot.drive.getAngularRate(0));
    m_driveTab.addNumber("Gyro Y", () -> Robot.drive.getAngularRate(1));
    m_driveTab.addNumber("Gyro Z", () -> Robot.drive.getAngularRate(2));

    // m_swerveModulesTab.addNumber("FL desired speed", () -> Robot.drive.swerveModuleStates[0].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("FR desired speed", () -> Robot.drive.swerveModuleStates[1].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BL desired speed", () -> Robot.drive.swerveModuleStates[2].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BR desired speed", () -> Robot.drive.swerveModuleStates[3].speedMetersPerSecond);

    // m_swerveModulesTab.addNumber("FL desired angle", () -> Robot.drive.swerveModuleStates[0].angle.getDegrees());
    // m_swerveModulesTab.addNumber("FR desired angle", () -> Robot.drive.swerveModuleStates[1].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BL desired angle", () -> Robot.drive.swerveModuleStates[2].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BR desired angle", () -> Robot.drive.swerveModuleStates[3].angle.getDegrees());

    m_swerveModulesTab.addNumber("FL FF", () -> Robot.drive.m_frontLeft.turnFeedforward);
    m_swerveModulesTab.addNumber("FR FF", () -> Robot.drive.m_frontRight.turnFeedforward);
    m_swerveModulesTab.addNumber("BL FF", () -> Robot.drive.m_backLeft.turnFeedforward);
    m_swerveModulesTab.addNumber("BR FF", () -> Robot.drive.m_backRight.turnFeedforward);

    m_swerveModulesTab.addNumber("FL PID Output", () -> Robot.drive.m_frontLeft.turnOutput);
    m_swerveModulesTab.addNumber("FR PID Output", () -> Robot.drive.m_frontRight.turnOutput);
    m_swerveModulesTab.addNumber("BL PID Output", () -> Robot.drive.m_backLeft.turnOutput);
    m_swerveModulesTab.addNumber("BR PID Output", () -> Robot.drive.m_backRight.turnOutput);


    m_swerveModulesTab.addNumber("Vel Front Right", () -> Robot.drive.m_frontRight.getDriveVelocity());
    m_swerveModulesTab.addNumber("Vel Front Left", () -> Robot.drive.m_frontLeft.getDriveVelocity());
    m_swerveModulesTab.addNumber("Vel Back Right", () -> Robot.drive.m_backRight.getDriveVelocity());
    m_swerveModulesTab.addNumber("Vel Back Left", () -> Robot.drive.m_backLeft.getDriveVelocity());
  }

}
