// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Interpolating.*;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX mainShooter = new WPI_TalonFX(Constants.mainShooterID);
  private WPI_TalonFX topShooter = new WPI_TalonFX(Constants.topShooterID);
  private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mainSpeedMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
  private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> topSpeedMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
  /** Creates a new Shooter. */
  public Shooter() {
    
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

    mainShooter.configOpenloopRamp(.5);
    topShooter.configOpenloopRamp(.5);

    mainShooter.setNeutralMode(NeutralMode.Coast);
    topShooter.setNeutralMode(NeutralMode.Coast);

    mainShooter.setInverted(false);
    topShooter.setInverted(true);

    //key = distance, value = speed
    //mainSpeedMap.put(key, value);  TODO
    //topSpeedMap.put(key, value);   TODO
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("mainShooter speed", mainShooter.getSelectedSensorVelocity());
    SmartDashboard.putNumber("topShooter speed", topShooter.getSelectedSensorVelocity());
  }
  /**
   * 
   * @param speedPreset
   * 0 = stop,
   * 1 = speed 1,
   * 2 = speed 2,
   * 3 = speed 3,
   * 4 = speed 4,
   * 5 = Interpolated Speed,
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
      case 4:
        mainShooter.set(TalonFXControlMode.Velocity, Constants.ShooterSpeed4);
        topShooter.set(TalonFXControlMode.Velocity, -(Constants.TopShooterSpeed4));
        break;
      case 5:
        mainShooter.set(TalonFXControlMode.Velocity, mainSpeedMap.getInterpolated(Camera.getDistance()).value);
        topShooter.set(TalonFXControlMode.Velocity, -topSpeedMap.getInterpolated(Camera.getDistance()).value);
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

  
}
