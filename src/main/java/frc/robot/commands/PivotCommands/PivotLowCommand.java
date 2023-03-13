package frc.robot.commands.PivotCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class PivotLowCommand extends CommandBase {
  private PivotSubsystem pivotSubsystem;
  private double setpoint;

  public PivotLowCommand(PivotSubsystem pivotSubs) {
    pivotSubsystem = pivotSubs;
    setpoint = -256000;
    addRequirements(pivotSubs);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());
    SmartDashboard.putBoolean("PIVOT DONE", false);
    pivotSubsystem.newSetpoint(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("PIVOT DONE", true);
  }

  @Override
  public boolean isFinished() {
    return pivotSubsystem.isAtSetpoint();
  }

}