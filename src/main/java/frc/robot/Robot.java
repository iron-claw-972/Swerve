// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.controls.Driver;
import frc.robot.controls.Operator;
import frc.robot.logging.DrivetrainIO;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIOReal;
import frc.robot.subsystems.drivetrain.DrivetrainIOSim;
import frc.robot.util.ShuffleboardManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autoCommand;
  public static ShuffleboardManager shuffleboard = new ShuffleboardManager();
  public static Drivetrain drive;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    if (isReal()) {
      drive = new Drivetrain(new DrivetrainIOReal());
    } else {
      drive = new Drivetrain(new DrivetrainIOSim());
    }

    setUseTiming(isReal()); // Run as fast as possible during replay
    LoggedNetworkTables.getInstance().addTable("/SmartDashboard"); // Log & replay "SmartDashboard" values (no tables are logged by default).
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
        Logger.getInstance().addDataReceiver(new ByteLogReceiver("/media/sda1/")); // Log to USB stick (name will be selected automatically)
        Logger.getInstance().addDataReceiver(new LogSocketServer(5800)); // Provide log data over the network, viewable in Advantage Scope.
    } else {
        String path = ByteLogReplay.promptForPath(); // Prompt the user for a file path on the command line
        Logger.getInstance().setReplaySource(new ByteLogReplay(path)); // Read log file for replay
        Logger.getInstance().addDataReceiver(new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim"))); // Save replay results to a new log with the "_sim" suffix
    }

    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    // This is really annoying so it's disabled 
    DriverStation.silenceJoystickConnectionWarning(true);

    shuffleboard.setup();

    Driver.configureControls();
    Operator.configureControls();

    drive.setDefaultCommand(new DefaultDriveCommand(
      drive,
      () -> Driver.getForwardTranslation() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> Driver.getSideTranslation() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> Driver.getRotation() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
    m_autoCommand = getAutonomousCommand();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link Robot} class.
   */
  @Override
  public void autonomousInit() {
    if (m_autoCommand != null) {
      m_autoCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return shuffleboard.getAutonomousCommand();
  }
}
