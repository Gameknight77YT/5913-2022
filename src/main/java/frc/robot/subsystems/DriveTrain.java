// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {
  WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.LeftMasterID);
  WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.RightMasterID);
  WPI_TalonFX leftSlave = new WPI_TalonFX(Constants.LeftSlaveID);
  WPI_TalonFX rightSlave = new WPI_TalonFX(Constants.RightSlaveID);

  DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.WheelBaseWith);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Robot.getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka);

  PIDController leftPidController = new PIDController(Constants.kp, Constants.ki, Constants.kd);
  PIDController rightPidController = new PIDController(Constants.kp, Constants.ki, Constants.kd);

  Pose2d pose = new Pose2d();
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightSlave.configFactoryDefault();

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);

    // invert setup
    leftMaster.setInverted(true);
    rightMaster.setInverted(false);
    leftSlave.setInverted(TalonFXInvertType.FollowMaster);
    rightSlave.setInverted(TalonFXInvertType.FollowMaster);


    // init encoders
    leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

    leftMaster.clearStickyFaults(10);
    rightMaster.clearStickyFaults(10);

    leftMaster.configOpenloopRamp(0.15);
    rightMaster.configOpenloopRamp(0.15);
    leftSlave.configOpenloopRamp(0.15);
    rightSlave.configOpenloopRamp(0.15);

    leftMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45, 175, 5));
    rightMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45, 175, 5));
    leftSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45, 175, 5));
    rightSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45, 175, 5));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(Robot.getHeading(),GetLeftMasterEncoderPose(),GetRightMasterEncoderPose());
    
  }

  public void resetOdometry(Pose2d pose2D) {
    ClearDriveEncoders();
    odometry.resetPosition(pose2D, Robot.getRotation2d());
  }
  
  public DifferentialDriveWheelSpeeds getSpeeds() {
   return new DifferentialDriveWheelSpeeds(
    leftMaster.getSelectedSensorVelocity() / Constants.GearRatio * (2 * Math.PI * Units.inchesToMeters(Constants.WheelRadiusInches)) / 60,
    rightMaster.getSelectedSensorVelocity() / Constants.GearRatio * (2 * Math.PI * Units.inchesToMeters(Constants.WheelRadiusInches)) / 60
   );//nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorVelocity()*10)
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedforward;
  }

  public PIDController getleftPidController() {
    return leftPidController;
  }
  
  public PIDController getrightPidController() {
    return rightPidController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return Kinematics;
  }

  public Pose2d getPose() {
     return odometry.getPoseMeters();
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), Robot.getHeading());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
  }

  

  /** Makes Robot Go Brrrrrrr */
  public void DriveWithJoystick(Joystick driverJoystick) {
    double joy_X = driverJoystick.getRawAxis(Constants.joystickX)*Constants.speedX;
    double joy_Y = -driverJoystick.getRawAxis(Constants.joystickY)*Constants.speedY;
    double threshold = .2;
    double leftMotorOutput;
    double rightMotorOutput;

    double xSpeed = MathUtil.clamp(joy_Y, -1.0, 1.0);
    xSpeed = applyDeadband(joy_Y, threshold);
    double zRotation = MathUtil.clamp(joy_X, -1.0, 1.0);
    zRotation = applyDeadband(joy_X, threshold);
    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }
    leftMaster.set(MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * 1);
    rightMaster.set(MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * 1);
  }
  
  /** Applys a Deadband */
  public double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public void Drive(double leftMotorOutput, double rightMotorOutput){
    leftMaster.set(leftMotorOutput);
    rightMaster.set(rightMotorOutput);
  }
  

  public void stopmotors(){
    rightMaster.stopMotor();
    leftMaster.stopMotor();
  }

  public double GetLeftMasterEncoderPose() {
    return nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition());
  }

  public double GetRightMasterEncoderPose() {
    return nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition());
  }

  public double GetLeftMasterEncoderSpeed() {
    return leftMaster.getSelectedSensorVelocity();
  }

  public double GetRightMasterEncoderSpeed() {
    return rightMaster.getSelectedSensorVelocity();
  }

  public void ClearDriveEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  /**sets neutral mode
   * 
   * @param mode 1 = coast, 0 = brake
   */
  public void SetMotorMode(double mode){
    if(mode==1){

    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);

    }else if(mode==0){

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
    }
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / 2048;
    double wheelRotations = motorRotations / Constants.GearRatio;// *?  TODO
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.WheelRadiusInches));
    return positionMeters;
  }
  
}
