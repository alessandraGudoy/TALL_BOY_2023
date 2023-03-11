package frc.robot;

import frc.robot.Constants.DriverControlConsts;
import frc.robot.commands.JoystickPivot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  PivotSubsystem pivot = new PivotSubsystem();
  XboxController xbox = new XboxController(0); 

  public RobotContainer() {
    pivot.setDefaultCommand(new JoystickPivot(pivot, () -> xbox.getLeftY()));
    configureBindings();
  }


  private void configureBindings() {
    new JoystickButton(xbox, 1).onTrue(new InstantCommand(() -> pivot.resetEnc()));

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
