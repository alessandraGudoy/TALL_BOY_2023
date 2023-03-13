package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.ClawCommands.Go90Clockwise;
import frc.robot.commands.ClawCommands.ToStartingPosition;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.PivotCommands.PivotLowCommand;
import frc.robot.commands.PivotCommands.PivotMiddleCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class HybridMobility extends SequentialCommandGroup {

  public HybridMobility(SwerveSubsystem swerve, PivotSubsystem pivot, ClawSubsystem claw) {

    // DROP IN HYBRID, GO BACK TO STARTING POSITION 
    addCommands(
      new Hybrid(swerve, pivot, claw),
      new DriveBackwardCommand(swerve, 27)

    );
  }
}
