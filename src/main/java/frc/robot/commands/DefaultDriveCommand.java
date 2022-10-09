package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DefaultDriveCommand extends CommandBase {
    private final Drivetrain m_drive;

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public DefaultDriveCommand(Drivetrain drive) {
        this.m_drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        driveWithJoystick(true);
    }

    private void driveWithJoystick(boolean fieldRelative) {

        double rawLeftY = Driver.getRawLeftY();
        double rawLeftX = Driver.getRawLeftX();
        double rawRightX = Driver.getRawRightX();

        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed =
            -m_xspeedLimiter.calculate(MathUtil.applyDeadband(Driver.getRawLeftY(), 0.02))
                * Constants.drive.kMaxSpeed;
    
        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed =
            -m_yspeedLimiter.calculate(MathUtil.applyDeadband(Driver.getRawLeftX(), 0.02))
                * Constants.drive.kMaxSpeed;
    
        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot =
            -m_rotLimiter.calculate(MathUtil.applyDeadband(Driver.getRawRightX(), 0.02))
                * Drivetrain.kMaxAngularSpeed;

        Logger.getInstance().recordOutput("DefaultDriveCommand/RawLeftY", rawLeftY);
        Logger.getInstance().recordOutput("DefaultDriveCommand/RawLeftX", rawLeftX);
        Logger.getInstance().recordOutput("DefaultDriveCommand/RawRightX", rawRightX);
        Logger.getInstance().recordOutput("DefaultDriveCommand/xSpeed", xSpeed);
        Logger.getInstance().recordOutput("DefaultDriveCommand/ySpeed", ySpeed);
        Logger.getInstance().recordOutput("DefaultDriveCommand/rot", rot);
    
        m_drive.drive(xSpeed, ySpeed, rot, fieldRelative);
      }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0.0, 0.0, 0.0, false);
    }
}
