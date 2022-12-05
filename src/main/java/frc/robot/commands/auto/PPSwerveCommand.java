package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.Functions;

/**
 * Custom PathPlanner version of SwerveControllerCommand
 */
public class PPSwerveCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final PathPlannerTrajectory trajectory;
    private final Supplier<Pose2d> poseSupplier;
    private final SwerveDriveKinematics kinematics;
    private final PPHolonomicDriveController controller;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final HashMap<String, Command> eventMap;
    private final Field2d field = new Field2d();

    private ArrayList<PathPlannerTrajectory.EventMarker> unpassedMarkers;

    /**
     * Constructs a new PPSwerveCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory         The trajectory to follow.
     * @param poseSupplier       A function that supplies the robot pose - use one of the odometry classes to provide this.
     * @param kinematics         The kinematics for the robot drivetrain.
     * @param xController        The Trajectory Tracker PID controller for the robot's x position.
     * @param yController        The Trajectory Tracker PID controller for the robot's y position.
     * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
     * @param outputModuleStates The raw output module states from the position controllers.
     * @param eventMap           Map of event marker names to the commands that should run when reaching that marker.
     *                           This SHOULD NOT contain any commands requiring the same subsystems as this command, or it will be interrupted
     * @param requirements       The subsystems to require.
     */
    public PPSwerveCommand(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            PIDController rotationController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            HashMap<String, Command> eventMap,
            Subsystem... requirements) {
        this.trajectory = trajectory;
        this.poseSupplier = poseSupplier;
        this.kinematics = kinematics;
        this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
        this.outputModuleStates = outputModuleStates;
        this.eventMap = eventMap;

        addRequirements(requirements);
    }

    /**
     * Constructs a new PPSwerveCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory         The trajectory to follow.
     * @param poseSupplier       A function that supplies the robot pose - use one of the odometry classes to provide this.
     * @param kinematics         The kinematics for the robot drivetrain.
     * @param xController        The Trajectory Tracker PID controller for the robot's x position.
     * @param yController        The Trajectory Tracker PID controller for the robot's y position.
     * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
     * @param outputModuleStates The raw output module states from the position controllers.
     * @param requirements       The subsystems to require.
     */
    public PPSwerveCommand(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            PIDController rotationController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Subsystem... requirements) {
        this(trajectory, poseSupplier, kinematics, xController, yController, rotationController, outputModuleStates, new HashMap<>(), requirements);
    }

    @Override
    public void initialize() {
        this.unpassedMarkers = new ArrayList<>();
        this.unpassedMarkers.addAll(this.trajectory.getMarkers());

        SmartDashboard.putData("PPSwerveCommand_field", this.field);
        this.field.getObject("traj").setTrajectory(this.trajectory);

        this.timer.reset();
        this.timer.start();

        PathPlannerServer.sendActivePath(this.trajectory.getStates());
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        PathPlannerState desiredState = (PathPlannerState) this.trajectory.sample(currentTime);

        Pose2d currentPose = this.poseSupplier.get();
        this.field.setRobotPose(currentPose);
        PathPlannerServer.sendPathFollowingData(new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation), currentPose);

        SmartDashboard.putNumber("PPSwerveCommand_xError", currentPose.getX() - desiredState.poseMeters.getX());
        SmartDashboard.putNumber("PPSwerveCommand_yError", currentPose.getY() - desiredState.poseMeters.getY());
        SmartDashboard.putNumber("PPSwerveCommand_rotationError", currentPose.getRotation().getRadians() - desiredState.holonomicRotation.getRadians());

        ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);
        SwerveModuleState[] targetModuleStates = this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

        this.outputModuleStates.accept(targetModuleStates);

        if(this.unpassedMarkers.size() > 0 && currentTime >= this.unpassedMarkers.get(0).timeSeconds) {
            PathPlannerTrajectory.EventMarker marker = this.unpassedMarkers.remove(0);

            if(this.eventMap.containsKey(marker.name)) {
                Command command = this.eventMap.get(marker.name);

                command.schedule();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();

        if(interrupted){
            this.outputModuleStates.accept(this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
        }
    }

    @Override
    public boolean isFinished() {
        /*
        return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds());
        return false;
        */
        double translationTolerance = 0.5;
        double rotationTolerence = 10;
        double timeTolerence = 2;
        
        Pose2d finalPose = this.trajectory.sample(this.trajectory.getTotalTimeSeconds()).poseMeters;
        Pose2d currentPose = this.poseSupplier.get();

        
        double deviationTranslation = Math.max(translationTolerance, translationTolerance * (this.timer.get() / this.trajectory.getTotalTimeSeconds()));
        double deviationRotation = Math.max(rotationTolerence, rotationTolerence * (this.timer.get() / this.trajectory.getTotalTimeSeconds()));

        boolean xReady = Functions.withinDeviation(finalPose.getX(), currentPose.getX(), deviationTranslation);
        boolean yReady =  Functions.withinDeviation(finalPose.getY(), currentPose.getY(), deviationTranslation);
        boolean rotReady = Functions.withinDeviation(finalPose.getRotation().getDegrees(), currentPose.getRotation().getDegrees(), deviationRotation);

        System.out.println(xReady + " " + yReady + " " + rotReady);
        if (this.timer.get() < this.trajectory.getTotalTimeSeconds() - timeTolerence) {
            return false;
        }else if (this.timer.get() > this.trajectory.getTotalTimeSeconds() + timeTolerence){
            return true;
        } else {
            return xReady && yReady && rotReady;
        }
        
    }
}
