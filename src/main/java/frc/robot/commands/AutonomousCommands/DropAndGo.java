package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawCommands.CloseClaw;
import frc.robot.commands.ClawCommands.OpenClaw;
import frc.robot.commands.MovementCommands.FieldBackward;
import frc.robot.commands.MovementCommands.FieldForward;
import frc.robot.commands.MovementCommands.FieldRotate;
import frc.robot.commands.MovementCommands.FieldStrafeRight;
import frc.robot.commands.PivotCommands.PivotLowCommand;
import frc.robot.commands.PivotCommands.PivotMiddleCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DropAndGo extends SequentialCommandGroup {
  public DropAndGo(SwerveSubsystem swerve, PivotSubsystem pivot, ClawSubsystem claw) {
    addCommands(

      //new ParallelCommandGroup(
        new OpenClaw(claw),
        new FieldForward(swerve, 255 / 3),
      //),

      new PivotLowCommand(pivot),

      new CloseClaw(claw),
      new Delay(0.5),

      new ParallelCommandGroup(
        new PivotMiddleCommand(pivot),
        new FieldBackward(swerve, 250 / 3)
      ),
      
      new Delay(0.2),
      new FieldStrafeRight(swerve, 24 / 3),

      new Delay(0.2),
      new FieldRotate(swerve, 180),

      new PivotLowCommand(pivot),

      new OpenClaw(claw)
    );
  }
}
