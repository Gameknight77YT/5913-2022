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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
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
  private WPI_TalonFX turretControl = new WPI_TalonFX(Constants.TurretControlID);
  private PIDController turretPidController = new PIDController(.1, 0, 0);

  private static final Pose2d targetPose = new Pose2d(new Translation2d(8.359, 4.125), new Rotation2d(0));
  private Pose2d robotPose;
  private double gx = targetPose.getX(),gy = targetPose.getY(),rx,ry;
  private double a,b,c;
  private double angleToGoal, heading, angleToTurnTo;
  private DriveTrain driveTrain;

  /** Creates a new Camera. */
  public Camera(DriveTrain dt) {
    driveTrain = dt; 
    CameraServer.startAutomaticCapture(1);
    turretControl.configFactoryDefault();
    turretControl.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    turretControl.clearStickyFaults(10);
    turretControl.setSelectedSensorPosition(0, 0, 10);
    turretControl.setSensorPhase(false);
    //turretControl.configMotionAcceleration(450, 10);
    //turretControl.configMotionCruiseVelocity(450, 10);
    turretControl.setNeutralMode(NeutralMode.Brake);
    turretControl.setInverted(true);
    turretControl.setSelectedSensorPosition(0);
    
  }

  @Override
  public void periodic() {

    turretControl.set(0); //why is this here
    // read values periodically
    final double x = tx.getDouble(0.0);
    final double y = ty.getDouble(0.0);
    final double area = ta.getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("turretEncoder", turretControl.getSelectedSensorPosition());

    
    Distance = new InterpolatingDouble((double)((int)((Constants.goalHeightfeet-Constants.limelightHeightFeet) / Math.tan(Units.degreesToRadians(Constants.limelightMountAngleDegrees+y)))));
    SmartDashboard.putNumber("Distance", Distance.value);
    
    limelightTracking();
    calcAimAngle();
    SmartDashboard.putNumber("turretMeters", nativeUnitsToDistanceMeters(getTurretEncoder()));
  }

  public void calcAimAngle(){
    robotPose = driveTrain.getPose();
    rx = robotPose.getX();
    ry = robotPose.getY();
    a = (gx-rx);
    b = (gy-ry);
    c = Math.sqrt(a*a + b*b);
    angleToGoal = Math.atan(a/b);
    heading += 90;
    heading = Robot.getHeading().getDegrees();
    while(heading >= 360 || heading <= -360){
      if(heading >= 360) heading -= 360;
      else if(heading <= -360) heading += -360;
    }
    if(heading <= -180) heading = 360 - Math.abs(heading);
    else if(heading > 180) heading = -360 + heading;
    angleToTurnTo = heading;
  }

  public void autoAimTurret(){
    turretControl.set(TalonFXControlMode.PercentOutput, turretPidController.calculate(turretEncoderInDegrees(), angleToTurnTo));
  }

  public double turretEncoderInDegrees(){
    double result = turretControl.getSelectedSensorPosition();
    result = nativeUnitsToDistanceMeters(result);
    result = result/(11.5/2);
    result = Units.radiansToDegrees(result);

    while(result >= 360 || result <= -360){
      if(result >= 360) result -= 360;
      else if(result <= -360) result += -360;
    }
    if(result <= -180) result = 360 - Math.abs(result);
    else if(result > 180) result = -360 + result;

    return result;
  }

  public static InterpolatingDouble getDistance() {
    return Distance;
  }

  public double getTurretEncoder(){
    return turretControl.getSelectedSensorPosition();
  }

  
  
  public void limelightTracking() {
    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.03; //0.04 how hard to turn toward the target
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


  public void Track(){//238690 //-111823
    ledMode.setValue(3);
      if (limelightHasValidTarget=true){
        Control(m_LimelightSteerCommand);
      }
      else{
        Control(0);
      }
    }

    public void Control(double speed){
      /*if(turretControl.getSelectedSensorPosition() >= -111823 && speed < 0){
      speed = 0;
    }else if(turretControl.getSelectedSensorPosition() <= 238690 && speed > 0){
      speed = 0;
      
    }else{

    }*/
      turretControl.set(TalonFXControlMode.PercentOutput, speed);
    }


    public void Reset(){
      ledMode.setNumber(3);
      Control(0);
    }


    public void ControlWithJoystick(Joystick manipulatorJoystick, double speed) {
      double input = manipulatorJoystick.getRawAxis(Constants.JoystickZAxisID);
      if(Math.abs(input) < .4){
        input=0;
       }
      if(manipulatorJoystick.getRawButton(Constants.TurnTurretRightButton)==true){
        Control(-speed );
      }else if(manipulatorJoystick.getRawButton(Constants.TurnTurretLeftButton)==true){
        Control(speed);
      }else{
        turretControl.stopMotor();
      }
        
      
    } 

    private double nativeUnitsToDistanceMeters(double sensorCounts){
      double motorRotations = (double)sensorCounts / 2048;
      double wheelRotations = motorRotations * 36*(187/25);// *?  TODO
      double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(11.5));//ish
      return positionMeters;
    }

}
      


