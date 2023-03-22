package frc.robot.commands.AutonomousCommands;

import java.util.concurrent.DelayQueue;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawCommands.CloseClaw;
import frc.robot.commands.ClawCommands.Go90Clockwise;
import frc.robot.commands.ClawCommands.Go90Counterclockwise;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.DriveForwardCommand;
import frc.robot.commands.MovementCommands.FieldBackward;
import frc.robot.commands.MovementCommands.FieldForward;
import frc.robot.commands.MovementCommands.FieldRotate;
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
      new FieldForward(swerve, 255 / 3),
      new Delay(0.15),
      new FieldRotate(swerve, 180),

      new ParallelCommandGroup(
        new PivotLowCommand(pivot),
        new Go90Clockwise(claw)
      ),
      
      new CloseClaw(claw),
      new Delay(0.5),

      new ParallelCommandGroup( 
        new PivotMiddleCommand(pivot),
        new Go90Counterclockwise(claw),
        new FieldBackward(swerve, 255 / 3)
      )
    );
  }
}
