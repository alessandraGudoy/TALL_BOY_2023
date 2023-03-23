package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawCommands.OpenClaw;
import frc.robot.commands.MovementCommands.FieldBackward;
import frc.robot.commands.MovementCommands.FieldForward;
import frc.robot.commands.MovementCommands.LevelOut;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DropAndStabilize extends SequentialCommandGroup {
  public DropAndStabilize(SwerveSubsystem swerve, PivotSubsystem pivot, ClawSubsystem claw) {
    addCommands(
      new OpenClaw(claw),
      new FieldForward(swerve, 250 / 3),
      new FieldBackward(swerve, 120 / 3)
      //new LevelOut(swerve),
      // new FieldForward(swerve, 3 / 3)
      
    );
  }
}
