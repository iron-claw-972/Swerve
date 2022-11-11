package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class FollowPathPlannerPath extends SequentialCommandGroup{
    Drivetrain m_drive; 
    PathPlannerTrajectory m_traj;
    boolean m_isFirstPath; 
    SwerveDriveKinematics m_kinematics; 
    public FollowPathPlannerPath(Drivetrain drive, PathPlannerTrajectory traj, boolean isFirstPath, SwerveDriveKinematics kinematics) {
        m_drive = drive; 
        m_traj = traj; 
        m_isFirstPath = isFirstPath;
        m_kinematics = kinematics; 
        addRequirements(drive);
        addCommands(
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                //TODO(MAYBE, ASK ANTHONY) RESET ODOMETRY FOR PIGEON
                //this.resetOdometry(traj.getInitialHolonomicPose());
              }),
            new PPSwerveControllerCommand(
                traj, 
                this::getPose, // Pose supplier
                SwerveDriveKinematics, // SwerveDriveKinematics
                new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                m_drive::setModuleStates, // Module states consumer
                //eventMap, // This argument is optional if you don't use event markers
                m_drive// Requires this drive subsystem
            );

        );
            
            
    
              
    }  
}
