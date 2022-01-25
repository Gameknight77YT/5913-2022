// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
   private WPI_TalonSRX climber = new WPI_TalonSRX(Constants.climberMotorID);
   private WPI_VictorSPX climerSlave = new WPI_VictorSPX(Constants.climberSlaveID);
   DoubleSolenoid swingSolenoid = new DoubleSolenoid(Constants.pcmID, PneumaticsModuleType.CTREPCM, Constants.swingForwardID, Constants.swingReverseID);
  /** Creates a new Climber. */
  public Climber() {
    climber.setInverted(false);
    climerSlave.setInverted(true);
    climerSlave.follow(climber);
    swingSolenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    climber.set(ControlMode.PercentOutput, speed);
  }
}
