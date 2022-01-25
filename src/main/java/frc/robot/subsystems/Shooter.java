// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  //private WPI_TalonSRX intake = new WPI_TalonSRX(Constants.intakeMotorID);
  //private WPI_TalonSRX feeder = new WPI_TalonSRX(Constants.feederMotorID);
  //private DoubleSolenoid intakeArms = new DoubleSolenoid(Constants.pcmID, PneumaticsModuleType.CTREPCM, Constants.intakeArmsForwardID, Constants.intakeArmsBackwardID);
  private WPI_TalonFX mainShooter = new WPI_TalonFX(Constants.mainShooterID);
  private WPI_TalonFX topShooter = new WPI_TalonFX(Constants.topShooterID);
  /** Creates a new Shooter. */
  public Shooter() {
    /*intake.setInverted(false);
    feeder.setInverted(false);

    intake.configOpenloopRamp(1);
    feeder.configOpenloopRamp(1);

    intake.clearStickyFaults(10);
    feeder.clearStickyFaults(10);

    intakeArms.set(Value.kReverse);*/
    
    /* Factory Default all hardware to prevent unexpected behaviour */
		mainShooter.configFactoryDefault();
		topShooter.configFactoryDefault();
		/* Config neutral deadband to be the smallest possible */
		mainShooter.configNeutralDeadband(0.001);
    topShooter.configNeutralDeadband(0.001);
		/* Config sensor used for Primary PID [Velocity] */
        mainShooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,10);
        topShooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,10);
    /* Config the peak and nominal outputs */
		mainShooter.configNominalOutputForward(0, 10);
		mainShooter.configNominalOutputReverse(0, 10);
		mainShooter.configPeakOutputForward(1, 10);
		mainShooter.configPeakOutputReverse(-1, 10);

    topShooter.configNominalOutputForward(0, 10);
		topShooter.configNominalOutputReverse(0, 10);
		topShooter.configPeakOutputForward(1, 10);
		topShooter.configPeakOutputReverse(-1, 10);
		/* Config the Velocity closed loop gains */
		mainShooter.config_kF(0, 1023.0/20660.0, 10);
		mainShooter.config_kP(0, 0.1, 10);
		mainShooter.config_kI(0, 0.001, 10);
		mainShooter.config_kD(0, 5, 10);

    topShooter.config_kF(0, 1023.0/20660.0, 10);
		topShooter.config_kP(0, 0.1, 10);
		topShooter.config_kI(0, 0.001, 10);
		topShooter.config_kD(0, 5, 10);
    
    mainShooter.setSelectedSensorPosition(0, 0, 10);
    topShooter.setSelectedSensorPosition(0, 0, 10);

    mainShooter.setNeutralMode(NeutralMode.Coast);
    topShooter.setNeutralMode(NeutralMode.Coast);

    mainShooter.setInverted(true);
    topShooter.setInverted(true);
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
        mainShooter.set(TalonFXControlMode.Velocity, Constants.ShooterSpeed1);
        topShooter.set(TalonFXControlMode.Velocity, -(Constants.TopShooterSpeed1));
        break;
      case 2:
        mainShooter.set(TalonFXControlMode.Velocity, Constants.ShooterSpeed2);
        topShooter.set(TalonFXControlMode.Velocity, -(Constants.TopShooterSpeed2));
        break;
      case 3:
        mainShooter.set(TalonFXControlMode.Velocity, Constants.ShooterSpeed3);
        topShooter.set(TalonFXControlMode.Velocity, -(Constants.TopShooterSpeed3));
        break;
    
      default:
        mainShooter.set(TalonFXControlMode.Velocity, otherValue);
        topShooter.set(TalonFXControlMode.Velocity, otherValue);
        break;
    }
  }

  public void stopShooter(){
    mainShooter.set(ControlMode.PercentOutput, 0);
    topShooter.set(ControlMode.PercentOutput, 0);
  }
  public void toggleIntakeArms(){
    //intakeArms.toggle();
  }

  public void controlIntake(double intakeSpeedPercent, double feederSpeedPercent){
    //intake.set(ControlMode.PercentOutput, intakeSpeedPercent);
    //feeder.set(ControlMode.PercentOutput, feederSpeedPercent);
  }
}
