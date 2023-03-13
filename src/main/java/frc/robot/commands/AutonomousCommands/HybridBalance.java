package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PitchBalance;
import frc.robot.commands.DriveCommands.Lock;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class HybridBalance extends SequentialCommandGroup {

  public HybridBalance(SwerveSubsystem swerve, PivotSubsystem pivot, ClawSubsystem claw) {

    // DROP IN HYBRID, GO BACK TO STARTING POSITION 
    addCommands(
      new Hybrid(swerve, pivot, claw),
      new PitchBalance(swerve), 
      new Lock(swerve)

    );
  }
}
