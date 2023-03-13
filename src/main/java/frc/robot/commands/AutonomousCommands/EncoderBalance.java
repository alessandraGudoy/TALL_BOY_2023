package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.Constants.SwerveConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class EncoderBalance extends CommandBase {
  private final SwerveSubsystem swerve; 

  public EncoderBalance(SwerveSubsystem newSwerve) {
    swerve = newSwerve; 

    addRequirements(newSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.driveBackward(AutoConsts.DRIVE_TRANSLATION_SPEED * 0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.getDriveEnc() < -1.5;
  }
}
