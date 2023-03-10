package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.ClawCommands.ToStartingPosition;
import frc.robot.commands.MovementCommands.DriveForwardCommand;
import frc.robot.commands.PivotCommands.PivotLowCommand;
import frc.robot.commands.PivotCommands.PivotMiddleCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Hybrid extends SequentialCommandGroup {

  public Hybrid(SwerveSubsystem swerve, PivotSubsystem pivot, ClawSubsystem claw) {

    // DROP IN HYBRID, GO BACK TO STARTING POSITION 
    addCommands(
      new ParallelCommandGroup(
        new ToStartingPosition(claw), 
        new PivotLowCommand(pivot)
      ), 

      new Claw(claw), 
      new PivotMiddleCommand(pivot)

    );
  }
}
