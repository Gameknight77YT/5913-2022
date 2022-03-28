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
  boolean isMainUpToSpeed = false;
  boolean isTopUpToSpeed = false;
  double top;
  double main;
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
		mainShooter.config_kF(0, .04625, 10);
		mainShooter.config_kP(0, .49875, 10);
		mainShooter.config_kI(0, 0, 10);
		mainShooter.config_kD(0, .5, 10);

    topShooter.config_kF(0, .04691, 10);
		topShooter.config_kP(0, .02791, 10);
		topShooter.config_kI(0, 0, 10);
		topShooter.config_kD(0, .2, 10);
    
    mainShooter.setSelectedSensorPosition(0, 0, 10);
    topShooter.setSelectedSensorPosition(0, 0, 10);

    mainShooter.configOpenloopRamp(.1);
    topShooter.configOpenloopRamp(.1);

    mainShooter.setNeutralMode(NeutralMode.Coast);
    topShooter.setNeutralMode(NeutralMode.Coast);

    mainShooter.setInverted(false);
    topShooter.setInverted(true);

    //key = distance, value = speed
    put(8, 8800, 5500);//   tarmac
    put(10, 8900, 6600);
    put(11, 8900, 7500);//  mid
    put(13, 8900, 9000);
    put(14, 9100, 11000);// launch pad 1
    put(16, 9500, 12000);
    put(18, 10000, 14000);//launch pad 2
  }

  /**
   * 
   * @param dis
   * @param main
   * @param top
   */
  private void put(int dis, int main, int top){
    put(dis+.0, main+.0, top+.0);
  }

  /**
   * 
   * @param dis
   * @param main
   * @param top
   */
  private void put(Double dis, Double main, Double top){
    InterpolatingDouble k = new InterpolatingDouble(dis);
    InterpolatingDouble mv = new InterpolatingDouble(main);
    InterpolatingDouble tv = new InterpolatingDouble(top);
    mainSpeedMap.put(k, mv);  
    topSpeedMap.put(k, tv);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("mainShooter speed", mainShooter.getSelectedSensorVelocity());
    SmartDashboard.putNumber("topShooter speed", topShooter.getSelectedSensorVelocity());
    SmartDashboard.putNumber("main", main);
    SmartDashboard.putNumber("top", top);
    //SmartDashboard.putNumber("mainInterpolation", mainSpeedMap.getInterpolated(Camera.getDistance()).value);
    //SmartDashboard.putNumber("topInterpolation", topSpeedMap.getInterpolated(Camera.getDistance()).value);
    main = SmartDashboard.getNumber("main", 0);
    top = SmartDashboard.getNumber("top", 0);
    SmartDashboard.putBoolean("isTopUpToSpeed", isTopUpToSpeed);
    SmartDashboard.putBoolean("isMainUpToSpeed", isMainUpToSpeed);
    isMainUpToSpeed = false;
    isTopUpToSpeed = false;
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
   * 6 = speed 6
   * other = other value
   * @param otherValue
   * if you want to use a different speed,
   * 0 otherwise
   */
  public void shootBall(int speedPreset, double otherValue){
    double mainSpeed;
    double topSpeed;
    switch (speedPreset) {
      case 1:
        mainSpeed = Constants.ShooterSpeed1;
        topSpeed = -(Constants.TopShooterSpeed1);
        break;
      case 2:
        mainSpeed = Constants.ShooterSpeed2;
        topSpeed = -(Constants.TopShooterSpeed2);
        break;
      case 3:
        mainSpeed = Constants.ShooterSpeed3;
        topSpeed = -(Constants.TopShooterSpeed3);
        break;
      case 4:
        mainSpeed = Constants.ShooterSpeed4;
        topSpeed = -(Constants.TopShooterSpeed4);
        break;
      case 5:
        mainSpeed = mainSpeedMap.getInterpolated(Camera.getDistance()).value;
        topSpeed = -topSpeedMap.getInterpolated(Camera.getDistance()).value;
        break;
      case 6:
        mainSpeed = Constants.ShooterSpeed6;
        topSpeed = -(Constants.TopShooterSpeed6);
        break;
      case 7:
        mainSpeed = 8900;
        topSpeed = -9000;
        break;
      default:
        mainSpeed = otherValue;
        topSpeed = otherValue;
        break;
    }

    
    mainShooter.set(ControlMode.Velocity, mainSpeed);
    topShooter.set(ControlMode.Velocity, topSpeed);

    if(mainSpeed-100 <= mainShooter.getSelectedSensorVelocity() && mainShooter.getSelectedSensorVelocity() <= mainSpeed+100){
      isMainUpToSpeed = true;
    }else isMainUpToSpeed = false;

    if(topSpeed-250 <= topShooter.getSelectedSensorVelocity() && topShooter.getSelectedSensorVelocity() <= topSpeed+250){
      isTopUpToSpeed = true;
    }else isTopUpToSpeed = false;
  }

  public void stopShooter(){
    mainShooter.set(ControlMode.PercentOutput, 0);
    topShooter.set(ControlMode.PercentOutput, 0);
  }

  
}
