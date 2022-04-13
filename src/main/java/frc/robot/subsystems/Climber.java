// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
   private WPI_TalonFX climber = new WPI_TalonFX(Constants.climberMasterID);
   private WPI_TalonFX climerSlave = new WPI_TalonFX(Constants.climberSlaveID);
   private String climberStatus = "normal";
  /** Creates a new Climber. */
  public Climber() {
    climerSlave.follow(climber);
    climber.setInverted(false);
    climerSlave.setInverted(TalonFXInvertType.OpposeMaster);
    
    climber.setNeutralMode(NeutralMode.Brake);
    climerSlave.setNeutralMode(NeutralMode.Brake);
    //climber.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Encoder",climber.getSelectedSensorPosition());
    SmartDashboard.putString("Climber status", climberStatus);
    SmartDashboard.putNumber("Climber speed", climber.getSelectedSensorVelocity());
    
  }

  

  public void clearClimbEncoder() {
    climber.setSelectedSensorPosition(0);
  }

  public double getClimbEncoder() {
    return climber.getSelectedSensorPosition();
  }

  public void setClimberMotor(double speed, Joystick driverJoystick){
    if(driverJoystick.getRawButton(Constants.climbUpButtonID)){
      // do nothing
    }else if(driverJoystick.getRawButton(Constants.climbDownButtonID)){
      speed = -speed;
    }else if(driverJoystick.getRawButton(Constants.climbUpSlowButtonID)){
      speed = speed/2;
    }else{
      speed = 0;
    }
    /*if(climber.getSelectedSensorPosition() <= Constants.climbEncoderBottom && speed < 0){
      speed = 0;
      climberStatus = "stoped at bottom";
    }else*/ if(climber.getSelectedSensorPosition() >= Constants.climbEncoderTop && speed > 0){
      speed = 0;
      climberStatus = "stoped at top";
    }else{
      climberStatus = "normal";
    }
    
    climber.set(TalonFXControlMode.PercentOutput, speed);
  }
}
