package frc.robot;

import frc.robot.Constants.DriverControlConsts;
import frc.robot.commands.PitchBalance;
import frc.robot.commands.AutonomousCommands.*;
import frc.robot.commands.ClawCommands.*;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.FieldBackward;
import frc.robot.commands.MovementCommands.FieldForward;
import frc.robot.commands.MovementCommands.FieldRotate;
import frc.robot.commands.PivotCommands.*;
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
  
  private XboxController xbox = new XboxController(DriverControlConsts.XBOX_CONTROLLER_PORT);
  private Joystick joystick = new Joystick(DriverControlConsts.JOYSTICK_PORT);

  //AUTONOMOUS CHOICES
  // 3 inches per encoder count
  // positive for left, negative for right => ROTATION
  private Command doNothing = new DoNothing();
  private Command hybridMobility = new HybridMobility(swerveSubsystem, pivotSubsystem, clawSubsystem);
  private Command hybridBalance = new HybridBalance(swerveSubsystem, pivotSubsystem, clawSubsystem);

  private Command dropAndGo = new DropAndGo(swerveSubsystem, pivotSubsystem, clawSubsystem);
  private Command dropAndStabilize = new DropAndStabilize(swerveSubsystem, pivotSubsystem, clawSubsystem);


  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new FieldOriented(swerveSubsystem,
        () -> xbox.getLeftY() * 0.95, // was negative
        () -> xbox.getLeftX() * 0.95, // was negative
        () -> xbox.getRightX() * 0.95));
        
    selectAuto();
    configureBindings();
  }


  private void configureBindings() {

    /* SWERVE */
    new JoystickButton(xbox, 1).toggleOnTrue(
        new FieldOriented(swerveSubsystem,
            () -> xbox.getLeftY() * 0.35,
            () -> xbox.getLeftX() * 0.35,
            () -> xbox.getRightX() * 0.35));

    new JoystickButton(xbox, 2).toggleOnTrue(new Lock(swerveSubsystem));
    new JoystickButton(xbox, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetNavx()));

    /* CLAW */
    new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));

    new JoystickButton(joystick, 4).onTrue(new Go90Clockwise(clawSubsystem));
    new JoystickButton(joystick, 6).onTrue(new ToStartingPosition(clawSubsystem));

    new JoystickButton(joystick, 2).whileTrue(new ManualClaw(clawSubsystem, () -> joystick.getX()));

    /* PIVOT */
     new JoystickButton(joystick, 5).onTrue(new PivotMiddleCommand(pivotSubsystem));
     new JoystickButton(joystick, 11).onTrue(new PivotLowCommand(pivotSubsystem));
     new JoystickButton(joystick, 1).whileTrue(new PivotJoystickCommand(pivotSubsystem, ()->joystick.getY()));

     new JoystickButton(xbox, 6).whileTrue(new PivotJoystickCommand(pivotSubsystem, ()->joystick.getY()));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void selectAuto() {
    autoChooser.setDefaultOption("Do Nothing", doNothing);
    autoChooser.addOption("Hybrid Mobility", hybridMobility);
    autoChooser.addOption("Hybrid Balance", hybridBalance);

    autoChooser.addOption("Drop and Go", dropAndGo);
    autoChooser.addOption("Drop and Stabilize", dropAndStabilize);

    SmartDashboard.putData(autoChooser);
  }

}
