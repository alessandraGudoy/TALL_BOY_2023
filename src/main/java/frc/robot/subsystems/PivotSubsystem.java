package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConsts;;

public class PivotSubsystem extends SubsystemBase{
    private WPI_TalonFX motor;
    private PIDController pid;
    private DigitalInput lowerLimit;
    private DigitalInput upperLimit;
    private boolean pidOn = true;
    private double setpoint;
    private double manualSpeed = 0;
    private double encoderValue;
    private TalonFXSensorCollection enc;
    
    public PivotSubsystem(){
        motor = new WPI_TalonFX(PivotConsts.PIVOT_MOTOR_PORT);
        pid = new PIDController(PivotConsts.PIVOT_KP, PivotConsts.PIVOT_KI, PivotConsts.PIVOT_KD);
        lowerLimit = new DigitalInput(PivotConsts.PIVOT_LOWER_LIMIT);
        upperLimit = new DigitalInput(PivotConsts.PIVOT_UPPER_LIMIT);
        enc = new TalonFXSensorCollection(motor);
        
        motor.setNeutralMode(NeutralMode.Brake);
    }

    /* * * PID Methods * * */
    public void enablePID(){
        pidOn = true;
    }

    public void disablePID(){
        pidOn = false;
    }

    public boolean isPIDOn(){
        return pidOn;
    }

    public void newSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    /* * * Encoder Methods * * */
    public double getEncoder(){
        return enc.getIntegratedSensorPosition();
    }

    public void resetEncoder(){
        enc.setIntegratedSensorPosition(0,0);
    }

    public void currentEnctoSetpoint(){
        setpoint = getEncoder();
    }

    /* * * Pivot Movement Methods * * */
    public void manualPivot(double speed){
        motor.set(speed);
    }

    public void pivotArm(double speed){
        motor.set(speed);
    }

    public void stopPivot(){
        motor.set(0);
    }

    public void setManualSpeed(double inputSpeed){
        manualSpeed = inputSpeed;
    }

    /* * * Limit Switch Methods * * */
    public boolean isLowerLimitPressed(){
        return lowerLimit.get();
    }

    public boolean isUpperLimitPressed(){
        return upperLimit.get();
    }

    public boolean isAtSetpoint(){
        double error = setpoint - getEncoder();
        return Math.abs(error) < 5;
    }

    @Override
    public void periodic(){
        encoderValue = getEncoder();
        // double calcSpeed = 0;

        // if(pidOn){
        //     calcSpeed = pid.calculate(encoderValue, setpoint);
        // }
        // else{
        //     calcSpeed = manualSpeed;
        // }
        // if(calcSpeed > 0){
        //     calcSpeed = 0;
        // }
        // else if(calcSpeed < 0){
        //     calcSpeed = 0;
        // }
        
        // motor.set(calcSpeed);

        SmartDashboard.putNumber("[P] ENCODER", getEncoder());
        SmartDashboard.putNumber("[P] SETPOINT", setpoint);
        SmartDashboard.putBoolean("[P] PID", isPIDOn());
        SmartDashboard.putBoolean("[P] LOW LIMIT", isLowerLimitPressed());
        SmartDashboard.putBoolean("[P] UPPER LIMIT", isUpperLimitPressed());
    }

}