package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class DefaultDriveCommand extends CommandBase {
    private final Drivetrain m_drive;

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(0);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(0);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(0);

    public DefaultDriveCommand(Drivetrain drive) {
        this.m_drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        driveWithJoystick(true);
    }

    private void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = MathUtil.applyDeadband(Driver.getRawLeftY(), Constants.oi.kDeadband)
                * Constants.drive.kMaxSpeed * 0.5;
    
        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = MathUtil.applyDeadband(Driver.getRawLeftX(), Constants.oi.kDeadband)
                * Constants.drive.kMaxSpeed * 0.5;
    
        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = MathUtil.applyDeadband(Driver.getRawRightX(), Constants.oi.kDeadband)
                * Constants.drive.kMaxAngularSpeed;
    
        m_drive.drive(xSpeed, ySpeed, rot, fieldRelative);
      }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0.0, 0.0, 0.0, false);
    }
}
