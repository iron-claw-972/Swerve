package frc.robot.controls;

import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);

  public static void configureControls() {
  }

  public static double getForwardTranslation() {
    return -Functions.expoMS(2, Functions.deadband(getRawLeftY(), 0.05));
  }

  public static double getSideTranslation() {
    return -Functions.expoMS(2, Functions.deadband(getRawLeftX(), 0.05));
  }

  public static double getRotation() {
    return -Functions.expoMS(2, Functions.deadband(getRawRightX(), 0.05));
  }

  public static double getRawRightX() {
    return driver.get(Axis.RIGHT_X);
  }

  public static double getRawLeftX() {
    return driver.get(Axis.LEFT_X);
  }

  public static double getRawLeftY() {
    return driver.get(Axis.LEFT_Y);
  }

}
