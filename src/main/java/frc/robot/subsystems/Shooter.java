// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonSRX intake = new TalonSRX(Constants.intakeMotorID);
  private TalonSRX feeder = new TalonSRX(Constants.feederMotorID);
  private DoubleSolenoid intakeArms = new DoubleSolenoid(Constants.pcmID, PneumaticsModuleType.CTREPCM, Constants.intakeArmsForwardID, Constants.intakeArmsBackwardID);
  WPI_TalonFX shooter = new WPI_TalonFX(Constants.shooterID);
  /** Creates a new Shooter. */
  public Shooter() {
    intake.setInverted(false);
    feeder.setInverted(false);

    intake.configOpenloopRamp(1);
    feeder.configOpenloopRamp(1);

    intake.clearStickyFaults(10);
    feeder.clearStickyFaults(10);

    intakeArms.set(Value.kReverse);
    
    /* Factory Default all hardware to prevent unexpected behaviour */
		shooter.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		shooter.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        shooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,10);

    /* Config the peak and nominal outputs */
		shooter.configNominalOutputForward(0, 10);
		shooter.configNominalOutputReverse(0, 10);
		shooter.configPeakOutputForward(1, 10);
		shooter.configPeakOutputReverse(-1, 10);

		/* Config the Velocity closed loop gains */
		shooter.config_kF(0, 1023.0/20660.0, 10);
		shooter.config_kP(0, 0.1, 10);
		shooter.config_kI(0, 0.001, 10);
		shooter.config_kD(0, 5, 10);
    
    shooter.setSelectedSensorPosition(0, 0, 10);

    shooter.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * 
   * @param speedPreset
   * 0 = stop,
   * 1 = speed 1,
   * 2 = speed 2,
   * 3 = speed 3,
   * other = other value
   * @param otherValue
   * if you want to use a different speed,
   * 0 otherwise
   */
  public void shootBall(int speedPreset, double otherValue){
    switch (speedPreset) {
      case 1:
        shooter.set(TalonFXControlMode.Velocity, Constants.ShooterSpeed1);
        break;
      case 2:
        shooter.set(TalonFXControlMode.Velocity, Constants.ShooterSpeed2);
        break;
      case 3:
        shooter.set(TalonFXControlMode.Velocity, Constants.ShooterSpeed3);
        break;
    
      default:
        shooter.set(TalonFXControlMode.Velocity, otherValue);
        break;
    }
  }

  public void stopShooter(){
    shooter.set(ControlMode.PercentOutput, 0);
  }
  public void toggleIntakeArms(){
    intakeArms.toggle();
  }

  public void controlIntake(double intakeSpeed, double feederSpeed){
    intake.set(ControlMode.PercentOutput, intakeSpeed);
    feeder.set(ControlMode.PercentOutput, feederSpeed);
  }
}
