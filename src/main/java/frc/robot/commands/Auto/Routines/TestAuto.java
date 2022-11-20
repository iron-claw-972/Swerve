package frc.robot.commands.auto.routines;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.subsystems.Drivetrain;

public class TestAuto extends SequentialCommandGroup{
   public TestAuto(){
    this(Robot.drive); 
   }

   public TestAuto(Drivetrain drive){
       addRequirements(drive);
       addCommands(
           new PathPlannerCommand("TestAuto", 0)
       );
   }

      
}
