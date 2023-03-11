package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveModule extends SubsystemBase{
    private CANSparkMax turningMotor;
    private CANSparkMax drivingMotor;

    private RelativeEncoder drivingEnc;
    private AbsoluteEncoder absoluteEncoder;

    private PIDController turningPID;

    private double chassisOffset;

    public SwerveModule(int turnPort, int drivePort, double chassisOffset, boolean driveReversed){
        turningMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
        drivingMotor = new CANSparkMax(drivePort, MotorType.kBrushless);

        absoluteEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(SwerveConsts.TURNING_ENCODER_ROTATION_CONVERSION);
        absoluteEncoder.setVelocityConversionFactor(SwerveConsts.TURNING_ENCODER_SPEED_CONVERSION);

        drivingEnc = drivingMotor.getEncoder();
        drivingEnc.setPositionConversionFactor(SwerveConsts.DRIVE_ENCODER_ROTATION_CONVERSION);
        drivingEnc.setVelocityConversionFactor(SwerveConsts.DRIVE_ENCODER_SPEED_CONVERSION);

        turningPID = new PIDController(SwerveConsts.KP_TURNING, SwerveConsts.KI_TURNING, SwerveConsts.KD_TURNING);
        turningPID.enableContinuousInput(-Math.PI, Math.PI); // System is circular;  Goes from -Math.PI to 0 to Math.PI

        this.chassisOffset = chassisOffset;
        
         drivingMotor.setInverted(driveReversed);
        absoluteEncoder.setInverted(true);

        drivingMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        resetEncoders();

    }

    /* * * ENCODER VALUES * * */

    public double getDrivePosition(){
        //return (drivingEnc.getPosition() % 900) * 360;
        return drivingEnc.getPosition();
    }

    //absolute encoder in radians 
    public double getAbsoluteEncoder(){
        return absoluteEncoder.getPosition() - chassisOffset;
    }

    /* * * SPEED VALUES * * */

    public double getDriveSpeed(){
        return drivingEnc.getVelocity();
    }

    // public double getTurningSpeed(){
    //     return turningEnc.getVelocity();
    // }

    //get the state of the module
    public SwerveModuleState getState(){
        // Rotation2d = rotation represented by a point on the unit circle
        // Rotation2d(double) => constructs a Rotation2d given the angle in radians
        
        // SwerveModuleState = state of a swerve module
        // SwerveModuleState(speed (in meters per second), angle of module (Using Rotation2d))
        return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getAbsoluteEncoder()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            drivingEnc.getPosition(),
            new Rotation2d(absoluteEncoder.getPosition() - chassisOffset));
    }

    // set turning enc to value of absolute encoder
    public void resetEncoders(){
        drivingEnc.setPosition(0);
        //turningEnc.setPosition(getAbsoluteEncoder());
    }

    //sets the desired state of the module (for left joystick)
    public void setDesiredState(SwerveModuleState state){
        // To make keep robot from going back to 0 position
        if(Math.abs(state.speedMetersPerSecond) < 0.1){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);

        // set speed
        drivingMotor.set(state.speedMetersPerSecond / SwerveConsts.MAX_SPEED * SwerveConsts.VOLTAGE); 
        turningMotor.set(turningPID.calculate(getAbsoluteEncoder(), state.angle.getRadians()));

        // Print to SmartDashboard
        // SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] drive speed", getDriveSpeed());
    }

    //sets the desired angle of the module (for right joystick)
    public void setAngle(SwerveModuleState state){

        state = SwerveModuleState.optimize(state, getState().angle);

        // set speed
        drivingMotor.set(0); 
        turningMotor.set(turningPID.calculate(getAbsoluteEncoder(), state.angle.getRadians()));

    }

    //stop modules 
    public void stop(){
        drivingMotor.set(0);
        turningMotor.set(0);
    }

    //set driving motor 
    public void setDrivingMotor(double speed){
        drivingMotor.set(speed);
    }

    @Override
    public void periodic(){
        int num = turningMotor.getDeviceId()+4;
        SmartDashboard.putNumber("S["+num+"] DRIVE SPEED", getDriveSpeed());
        SmartDashboard.putNumber("S["+num+"] ABS ENC", getAbsoluteEncoder());
        SmartDashboard.putNumber("S["+num+"] ENCODER", getDrivePosition());
    }

}