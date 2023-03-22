package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConsts;
import frc.robot.commands.PitchBalance;
import frc.robot.commands.ClawCommands.Go90Clockwise;
import frc.robot.commands.ClawCommands.Go90Counterclockwise;
import frc.robot.commands.ClawCommands.OpenClaw;
import frc.robot.commands.DriveCommands.Lock;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.PivotCommands.PivotLowCommand;
import frc.robot.commands.PivotCommands.PivotMiddleCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class HybridBalance extends SequentialCommandGroup {

  public HybridBalance(SwerveSubsystem swerve, PivotSubsystem pivot, ClawSubsystem claw) {

    // DROP IN HYBRID, GO BACK TO STARTING POSITION 
    addCommands(
      new ParallelCommandGroup(
        new PivotLowCommand(pivot), 
        new Go90Clockwise(claw)
      ),

      new OpenClaw(claw), 

      new Delay(1.0),

      new ParallelCommandGroup(
        new PivotMiddleCommand(pivot), 
        new Go90Counterclockwise(claw),
        new DriveBackwardCommand(swerve, 26/3) // 26 inches exact
      )
    );
  }
}
