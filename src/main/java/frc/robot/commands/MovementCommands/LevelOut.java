package frc.robot.commands.MovementCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class LevelOut extends CommandBase {
  private SwerveSubsystem swerve;

  public LevelOut(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(swerve.getRoll() < 2){
      swerve.driveBackward(0.05);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //swerve.stopModules();
    swerve.lock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.getRoll() > 2;//Timer.getMatchTime()<1.0;
  }
}
