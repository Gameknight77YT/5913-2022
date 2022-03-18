// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Interpolating.InterpolatingDouble;

public class Camera extends SubsystemBase {

  //networktables
  final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  final NetworkTableEntry tx = table.getEntry("tx");
  final NetworkTableEntry ty = table.getEntry("ty"); 
  final NetworkTableEntry ta = table.getEntry("ta");
  final NetworkTableEntry ledMode = table.getEntry("ledMode");

  private static InterpolatingDouble Distance;

  boolean limelightHasValidTarget = false;
  private double m_LimelightSteerCommand = 0.0;
  WPI_TalonFX turretControl = new WPI_TalonFX(Constants.TurretControlID);

  /** Creates a new Camera. */
  public Camera() {
    CameraServer.startAutomaticCapture(0);

    turretControl.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    turretControl.clearStickyFaults(10);
    turretControl.setSelectedSensorPosition(0, 0, 10);
    turretControl.setSensorPhase(false);
    turretControl.configMotionAcceleration(450, 10);
    turretControl.configMotionCruiseVelocity(450, 10);
    turretControl.setNeutralMode(NeutralMode.Coast);
    turretControl.setInverted(true);
  }

  @Override
  public void periodic() {
    turretControl.set(0);
    // read values periodically
    final double x = tx.getDouble(0.0);
    final double y = ty.getDouble(0.0);
    final double area = ta.getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    
    Distance = new InterpolatingDouble((double)((int)((Constants.goalHeightfeet-Constants.limelightHeightFeet) / Math.tan(Units.degreesToRadians(Constants.limelightMountAngleDegrees+y)))));
    SmartDashboard.putNumber("Distance", Distance.value);
    

    limelightTracking();
  }

  public static InterpolatingDouble getDistance() {
    return Distance;
  }

  
  
  public void limelightTracking() {
    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.08; //0.04 how hard to turn toward the target
    final double min_command = 0.02;
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    if (tv < 1.0){
      limelightHasValidTarget = false;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    limelightHasValidTarget = true;

    // Start with proportional steering
   double steer_cmd = 0.0;
   
   if (tx < 0) {
    steer_cmd = tx * STEER_K - min_command;
   }
   else if (tx > 0) {
    steer_cmd = tx * STEER_K + min_command;
   }
    m_LimelightSteerCommand = steer_cmd * -1;

    }

    public void AutoTrack(){
      limelightTracking();
      turretControl.set(ControlMode.PercentOutput, m_LimelightSteerCommand);
    }

  public void Track(){
    ledMode.setValue(0);
      if (limelightHasValidTarget=true){
        turretControl.set(ControlMode.PercentOutput, m_LimelightSteerCommand);
      }
      else{
        turretControl.set(ControlMode.PercentOutput, 0);
      }
    }

    public void Control(double speed){
      turretControl.set(TalonFXControlMode.PercentOutput, speed);
    }


    public void Reset(){
      ledMode.setNumber(0);
      turretControl.set(ControlMode.PercentOutput, 0);
    }


    public void ControlWithJoystick(Joystick manipulatorJoystick, double speed) {
      double input = manipulatorJoystick.getRawAxis(Constants.JoystickZAxisID);
      if(Math.abs(input) < .4){
        input=0;
       }
      if(manipulatorJoystick.getRawButton(Constants.ActivateTurnTurretButton)==true){
        turretControl.set(ControlMode.PercentOutput, input*-speed );
      }else{
        turretControl.stopMotor();
      }
        
      
    } 

}
      


