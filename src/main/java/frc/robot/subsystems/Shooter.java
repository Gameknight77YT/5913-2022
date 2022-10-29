// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Interpolating.*;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX mainShooter = new WPI_TalonFX(Constants.mainShooterID);
  private WPI_TalonFX topShooter = new WPI_TalonFX(Constants.topShooterID);
  private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mainSpeedMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
  private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> topSpeedMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
  private CANdle candle = new CANdle(Constants.CANdleID);
  public boolean isMainUpToSpeed = false;
  public boolean isTopUpToSpeed = false;
  public boolean isMainSpeedingUp = false;
  public boolean isTopSpeedingUp = false;

  //private ColorSensorV3 colorSensor = null;//new ColorSensorV3(I2C.Port.kOnboard);//TODO
  /** Creates a new Shooter. */
  public Shooter() {
    candle.configFactoryDefault();
    candle.configLOSBehavior(false);
    candle.configLEDType(LEDStripType.GRB);
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

    
    put(9, 8900, 5700);//   tarmac
    put(11, 9200, 6800);
    put(12, 9250, 7700);//  mid
    put(13, 9550, 8300);
    put(14, 9500, 10000);
    put(15, 9550, 11100);// launch pad 1
    put(16, 9600, 11500);
    put(17, 10200, 12500);
    put(19, 10000, 14000);//launch pad 2
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
    //key = distance, value = speed
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
    //SmartDashboard.putNumber("mainInterpolation", mainSpeedMap.getInterpolated(Camera.getDistance()).value);
    //SmartDashboard.putNumber("topInterpolation", topSpeedMap.getInterpolated(Camera.getDistance()).value);
    SmartDashboard.putBoolean("isTopUpToSpeed", isTopUpToSpeed);
    SmartDashboard.putBoolean("isMainUpToSpeed", isMainUpToSpeed);
    isMainUpToSpeed = false;
    isTopUpToSpeed = false;
    isMainSpeedingUp = false;
    isTopSpeedingUp = false;
    
    //SmartDashboard.putString("colorSensor", Color());//TODO
  }

  /*public String Color(){ //TODO
    Color color = colorSensor.getColor();
    String output = null;
    if(color.blue > .25 || color.red > .25){
      if(color.blue > color.red){
        output = "blueBall";
      }else if(color.blue < color.red){
        output = "redBall";
      }
    }else{
      output = "noBall";
    }
    return output;
  }*/

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
        mainSpeed = 1.08 * mainSpeedMap.getInterpolated(Camera.getDistance()).value;
        topSpeed = 1.08 * -topSpeedMap.getInterpolated(Camera.getDistance()).value;
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

    isMainSpeedingUp = true;
    isTopSpeedingUp = true;

    mainShooter.set(ControlMode.Velocity, mainSpeed);
    topShooter.set(ControlMode.Velocity, topSpeed);

    if(mainSpeed-100 <= mainShooter.getSelectedSensorVelocity() && mainShooter.getSelectedSensorVelocity() <= mainSpeed+100){
      isMainUpToSpeed = true;
    }else isMainUpToSpeed = false;

    if(topSpeed-800 <= topShooter.getSelectedSensorVelocity() && topShooter.getSelectedSensorVelocity() <= topSpeed + 800){
      isTopUpToSpeed = true;
    }else isTopUpToSpeed = false;
  }
  

  public void stopShooter(){
    mainShooter.set(ControlMode.PercentOutput, 0);
    topShooter.set(ControlMode.PercentOutput, 0);
    
  }

  public void setLEDs(int r, int g, int b){
    candle.setLEDs(r, g, b);
  }

  public void setAnimation(Animation animation){
    candle.animate(animation);
  }
  
}
