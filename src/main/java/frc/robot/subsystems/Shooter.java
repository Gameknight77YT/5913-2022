// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  WPI_TalonFX shooter = new WPI_TalonFX(Constants.shooterID);
  /** Creates a new Shooter. */
  public Shooter() {
    shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooter.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * 
   * @param speedPreset
   * 1 = speed 1
   * 2 = speed 2
   * 3 = speed 3
   * other = other value
   * @param otherValue
   * if you want to use a different speed,
   * 0 otherwise
   */
  public void shootBall(int speedPreset, double otherValue){
    switch (speedPreset) {
      case 1:
        shooter.set(ControlMode.Velocity, Constants.ShooterSpeed1);
        break;
      case 2:
        shooter.set(ControlMode.Velocity, Constants.ShooterSpeed2);
        break;
      case 3:
        shooter.set(ControlMode.Velocity, Constants.ShooterSpeed3);
        break;
    
      default:
        shooter.set(ControlMode.Velocity, otherValue);
        break;
    }
  }
}
