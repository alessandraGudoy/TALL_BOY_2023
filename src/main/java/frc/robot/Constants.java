package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class DriverControlConsts {
    public static final int XBOX_CONTROLLER_PORT = 0;
    public static final int JOYSTICK_PORT = 1;
  }

  public static class SwerveConsts{
    /* * * MEASUREMENTS * * */
    public static final double WHEEL_DIAMETER = 3 * 2.5 / 100;
    public static final double TRACK_WIDTH = 0.435;
    public static final double WHEEL_BASE = 0.435;

    public static final double DRIVING_REDUCTION = (45.0 * 22) / (14 * 15);

    public static final double GEAR_RATIO = 4.71 / 1;
    public static final double STEER_GEAR_RATIO = 150 / 7; //FIXME

    public static final double VOLTAGE = 8;

    /* * * SWERVE DRIVE KINEMATICS * * */
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      // front left
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // back left
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // back right
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      // front right
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    /* * * FRONT LEFT * * */
    public static final int FL_DRIVE_PORT = 1;
    public static final int FL_TURN_PORT = 5;
    public static final double FL_OFFSET = -Math.PI / 2; //-Math.PI / 2;

    /* * * BACK LEFT * * */
    public static final int BL_DRIVE_PORT = 2;
    public static final int BL_TURN_PORT = 6;
    public static final double BL_OFFSET = -Math.PI; //Math.PI;

    /* * * BACK RIGHT * * */
    public static final int BR_DRIVE_PORT = 3;
    public static final int BR_TURN_PORT = 7;
    public static final double BR_OFFSET = Math.PI / 2; //Math.PI / 2;

    /* * * FRONT RIGHT * * */
    public static final int FR_DRIVE_PORT = 4;
    public static final int FR_TURN_PORT = 8;
    public static final double FR_OFFSET = -Math.toRadians(0);

    /* * * CONVERSIONS FOR ENCODERS * * */
    public static final double ABSOLUTE_ENCODER_ROTATION_CONVERSION = 1;
    public static final double ABSOLUTE_ENCODER_SPEED_CONVERSION = ABSOLUTE_ENCODER_ROTATION_CONVERSION / 60;
    // 8192 counts per motor revolution
    // 8192 * steer gear ratio  => counts per wheel revolution
    
    public static final double DRIVE_ENCODER_ROTATION_CONVERSION = (WHEEL_DIAMETER * Math.PI) / DRIVING_REDUCTION;;
    public static final double DRIVE_ENCODER_SPEED_CONVERSION = DRIVE_ENCODER_ROTATION_CONVERSION / 60;

    public static final double TURNING_ENCODER_ROTATION_CONVERSION = 2.0 * Math.PI;
    public static final double TURNING_ENCODER_SPEED_CONVERSION = TURNING_ENCODER_ROTATION_CONVERSION / 60;
      // 5676 / 60 * 341.88 * Math.PI

    /* * * MAX SPEEDS * * */
    public static final double MAX_SPEED = 4.804;
      // 5676.0 / 60.0 * ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)) *
      // 0.10033 * Math.PI; // 13.5 feet per second = 4.1148 meters per second
    public static final double MAX_ROTATION = MAX_SPEED / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

    /* * * PID VALUES * * */
    public static final double KP_TURNING = 0.18;
    public static final double KI_TURNING = 0.0;
    public static final double KD_TURNING = 0.0002;
  }

  public static class PivotConsts {
    public static final int PIVOT_MOTOR_PORT = 14;
    public static final int PIVOT_LOWER_LIMIT = 9;
    public static final int PIVOT_UPPER_LIMIT = 8;

    public static final double PIVOT_KP = 0.00003;
    public static final double PIVOT_KI = 0.0;//0.000001
    public static final double PIVOT_KD = 0.0;
  }

  public static class ClawConsts {
    public static final int WRIST_MOTOR_PORT = 15;
    public static final int CLAW_FORWARD_CHANNEL = 0;
    public static final int CLAW_REVERSE_CHANNEL = 1;

    public static final double WRIST_SPEED = 0.8;

    public static final double ROTATE_90 = 100;
    public static final double ROTATION_T0_90_ENC = 100;
    public static final double ROTATION_TO_OTHER_90_ENC = -100;
  }

  public static class AutoConsts {
    public static final double DRIVE_TRANSLATION_SPEED = 0.25;
    public static final double DRIVE_ROTATION_SPEED = 0.25;
  }

}
