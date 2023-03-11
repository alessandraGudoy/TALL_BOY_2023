// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
  WPI_TalonFX pivot; 
  TalonFXSensorCollection enc;

  public PivotSubsystem() {
    pivot = new WPI_TalonFX(4); 
    enc = new TalonFXSensorCollection(pivot);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pivot enc", enc.getIntegratedSensorPosition());
    SmartDashboard.putNumber("pivot velocity", enc.getIntegratedSensorVelocity());
  }

  public void setSpeed(double speed) {
    pivot.set(speed);
  }

  public void resetEnc() {
    enc.setIntegratedSensorPosition(0, 0);
  }
}
