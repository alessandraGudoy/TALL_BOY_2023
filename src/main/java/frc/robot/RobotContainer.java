package frc.robot;

import frc.robot.Constants.DriverControlConsts;
import frc.robot.commands.AutonomousCommands.*;
import frc.robot.commands.ClawCommands.*;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.PivotCommands.*;
import frc.robot.commands.LED_Commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  public static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static PivotSubsystem pivotSubsystem = new PivotSubsystem();
  public static ClawSubsystem clawSubsystem = new ClawSubsystem();
  public static Lights lights = new Lights();

  private XboxController xbox = new XboxController(DriverControlConsts.XBOX_CONTROLLER_PORT);
  private Joystick joystick = new Joystick(DriverControlConsts.JOYSTICK_PORT);

  //AUTONOMOUS CHOICES
  private Command doNothing = new DoNothing();
  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new DriverControl(swerveSubsystem,
        () -> xbox.getLeftY() * 0.95,
        () -> xbox.getLeftX() * 0.95,
        () -> xbox.getRightX() * 0.95));
        
    selectAuto();
    configureBindings();
  }


  private void configureBindings() {

    /* SWERVE */
    new JoystickButton(xbox, 1).toggleOnTrue(
        new DriverControl(swerveSubsystem,
            () -> -xbox.getLeftY() * 0.35,
            () -> -xbox.getLeftX() * 0.35,
            () -> -xbox.getRightX() * 0.35));
    // new JoystickButton(xbox, 6).toggleOnTrue(
    //     new DriverControl(swerveSubsystem,
    //         () -> -xbox.getLeftY() * 0.75,
    //         () -> -xbox.getLeftX() * 0.75,
    //         () -> -xbox.getRightX() * 0.75));
    //new JoystickButton(xbox, 2).toggleOnTrue(new Lock(swerveSubsystem));

    new JoystickButton(xbox, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetNavx()));

    /* CLAW */
    // new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));

    new JoystickButton(xbox, 2).onTrue(new Go90Clockwise(clawSubsystem));
    new JoystickButton(xbox, 4).onTrue(new ToStartingPosition(clawSubsystem));
    new JoystickButton(xbox, 3).onTrue(new Go90Counterclockwise(clawSubsystem));

    new JoystickButton(joystick, 2).whileTrue(new ManualClaw(clawSubsystem, () -> joystick.getX()));

    /* PIVOT */
    //  new JoystickButton(joystick, 4).onTrue(new PivotMiddleCommand(pivotSubsystem));
    //  new JoystickButton(joystick, 3).onTrue(new PivotLowCommand(pivotSubsystem));
    //  new JoystickButton(joystick, 5).whileTrue(new PivotJoystickCommand(pivotSubsystem, ()->xbox.getRightY()));

    /* LIGHTS */
    new JoystickButton(joystick, 6).toggleOnTrue(new Yellow(lights));
    new JoystickButton(joystick, 4).toggleOnTrue(new Violet(lights));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void selectAuto() {
    autoChooser.setDefaultOption("Do Nothing", doNothing);

    SmartDashboard.putData(autoChooser);
  }

}
