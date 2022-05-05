// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  private WPI_TalonFX intake = new WPI_TalonFX(Constants.intakeMotorID);
  private WPI_TalonFX starfishWheels = new WPI_TalonFX(Constants.starfishWheelsMotorID);
  private WPI_TalonSRX feeder = new WPI_TalonSRX(Constants.feederMotorID);
  private DoubleSolenoid intakeArms = new DoubleSolenoid(Constants.pcmID, PneumaticsModuleType.CTREPCM, Constants.intakeArmsForwardID, Constants.intakeArmsBackwardID);
  private Compressor compressor = new Compressor(Constants.pcmID, PneumaticsModuleType.CTREPCM);
  /** Creates a new Intake. */
  public Intake() {
    compressor.enableDigital();
    intake.configFactoryDefault();
    starfishWheels.configFactoryDefault();
    feeder.configFactoryDefault();

    intake.setInverted(true);
    starfishWheels.setInverted(true);
    feeder.setInverted(false);

    intake.clearStickyFaults(10);
    starfishWheels.clearStickyFaults(10);
    feeder.clearStickyFaults(10);

    /*
    intake.configOpenloopRamp(0);
    starfishWheels.configOpenloopRamp(0);
    feeder.configOpenloopRamp(0);
    */

    //intakeArms.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void toggleIntakeArms(){
    intakeArms.toggle();
  }

  /**
   * 
   * @param i 1 = forward, 2 = reverse
   */
  public void toggleIntakeArms(int i){
    switch (i) {
      case 1:
        intakeArms.set(Value.kForward);
        break;
      case 2:
        intakeArms.set(Value.kReverse);
        break;
      default:
        intakeArms.toggle();
        break;
    }
  }

  public void stopCompressor(){
    compressor.disable();
  }

  public void controlIntake(double intakeSpeedPercent, double starfishWheelsSpeed, double feederSpeedPercent){
    intake.set(ControlMode.PercentOutput, intakeSpeedPercent);
    starfishWheels.set(ControlMode.PercentOutput, starfishWheelsSpeed);
    feeder.set(ControlMode.PercentOutput, feederSpeedPercent);
  }
  
  public void stopIntake(){
    intake.set(ControlMode.PercentOutput, 0);
    starfishWheels.set(ControlMode.PercentOutput, 0);
    feeder.set(ControlMode.PercentOutput, 0);
    compressor.enableDigital();
  }

}
