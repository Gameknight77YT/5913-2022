// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
   private WPI_TalonFX climber = new WPI_TalonFX(Constants.climberMotorID);
   private WPI_TalonFX climerSlave = new WPI_TalonFX(Constants.climberSlaveID);
   DoubleSolenoid swingSolenoid = new DoubleSolenoid(Constants.pcmID, PneumaticsModuleType.CTREPCM, Constants.swingForwardID, Constants.swingReverseID);
   private String climberStatus = "normal";
  /** Creates a new Climber. */
  public Climber() {
    climber.setInverted(false);
    climerSlave.setInverted(true);
    climerSlave.follow(climber);
    swingSolenoid.set(Value.kForward);
    climber.setNeutralMode(NeutralMode.Brake);
    climerSlave.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Encoder",climber.getSelectedSensorPosition());
    SmartDashboard.putString("Climber status", climberStatus);
    SmartDashboard.putNumber("Climber speed", climber.getSelectedSensorVelocity());
    
  }

  public void swingOut() {
    swingSolenoid.set(Value.kReverse);
  }

  public void swingIn() {
    swingSolenoid.set(Value.kForward);
  }

  public void clearClimbEncoder() {
    climber.setSelectedSensorPosition(0);
  }

  public double getClimbEncoder() {
    return climber.getSelectedSensorPosition();
  }

  public void setClimberMotor(double speed){
    /*if(climber.getSelectedSensorPosition() <= 0 && speed < 0){
      speed = 0;
      climberStatus = "stoped at bottom";
    }else if(climber.getSelectedSensorPosition() >= Constants.climbEncoderTop && speed > 0){
      speed = 0;
      climberStatus = "stoped at top";
    }else{
      climberStatus = "normal";
    }*/
    climber.set(TalonFXControlMode.PercentOutput, speed);
  }
}
