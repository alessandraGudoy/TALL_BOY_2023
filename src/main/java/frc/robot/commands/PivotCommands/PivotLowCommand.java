package frc.robot.commands.PivotCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class PivotLowCommand extends CommandBase {
  private PivotSubsystem pivotSubsystem;
  private double setpoint;

  public PivotLowCommand(PivotSubsystem pivotSubs) {
    pivotSubsystem = pivotSubs;
    setpoint = 0; //FIXME
    addRequirements(pivotSubs);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    pivotSubsystem.newSetpoint(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return pivotSubsystem.isAtSetpoint();
  }

}