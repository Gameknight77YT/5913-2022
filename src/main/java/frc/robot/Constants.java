// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // can IDs
    public static final int LeftMasterID = 1;
    public static final int RightMasterID = 2;
    public static final int LeftSlaveID = 3;
    public static final int RightSlaveID = 4;
    public static final int mainShooterID = 5;
    public static final int topShooterID = 6;
    public static final int climberMasterID = 7;
    public static final int climberSlaveID = 8;
    public static final int intakeMotorID = 9;
    public static final int feederMotorID = 10;
    public static final int starfishWheelsMotorID = 11;
    public static final int TurretControlID = 12;
    public static final int pcmID = 13;
    public static final int pdpID = 14;
    public static final int CANdleID = 15;

    //solinoid ids 
    public static final int swingReverseID = 0;
    public static final int swingForwardID = 1;
    public static final int intakeArmsBackwardID = 2;
    public static final int intakeArmsForwardID = 3;
    public static final int hookReverseID = 4;
    public static final int hookForwardID = 5;

    //joystick ids
    public static final int driverJoystickID = 0;
    public static final int manipulatorJoystickID = 1;
    public static final int joystickX = 0;
    public static final int joystickY = 1;
    public static final int JoystickZAxisID = 2;

    //speed  constants
    public static final double speedX = .95;
    public static final double speedY = .95;
    public static final double TurnTurretSpeed = .95;
    public static final double climberSpeed = 1;
    public static final double ShooterSpeed1 = 8800;// no
    public static final double TopShooterSpeed1 = 5500;//no
    public static final double ShooterSpeed2 = 3500;// 7 
    public static final double TopShooterSpeed2 = 3500;// 7 
    public static final double ShooterSpeed3 = 9000;// 12 
    public static final double TopShooterSpeed3 = 10000;// 12 
    public static final double ShooterSpeed4 = 8750;// 11 
    public static final double TopShooterSpeed4 = 7000;// 11
    public static final double ShooterSpeed6 = 10000; //no 
    public static final double TopShooterSpeed6 = 14000; //no
    public static final double intakeSpeed = .95;
    public static final double feederSpeed = .50;
    public static final double starfishSpeed = .50;
    public static final double autoSpeed = .95;

    //times / encoder counts
    public static final double stopAndShootTime = 1;
    public static final double climbEncoderTop = 336549;//339242
    public static final double climbEncoderBottom = -7000;

    //buttons driver
    public static final int climbUpButtonID = 6;
    public static final int climbDownButtonID = 4;
    public static final int intakeArmsOutButtonID = 1; 
    public static final int intakeArmsInButtonID = 2;
    public static final int climbUpSlowButtonID = 7;
    public static final int outTakeBallButtonID = 5;
    public static final int outTakeBall2ButtonID = 11;
    public static final int intakeBallButtonID = 3; 

    //manipulator 
    public static final int hooksInButtonID = 10;
    public static final int hooksOutButtonID = 8;
    public static final int swingInButtonID = 4;
    public static final int swingOutButtonID = 3;
    public static final int shootBallAutoSpeedButtonID = 9;
    public static final int shootBall4ButtonID = 11;
    public static final int shootBall3ButtonID = 12;
    public static final int shootBall2ButtonID = 7; 
    public static final int shootBall1ButtonID = 11; //no
    public static final int shootBall6ButtonID = 12;//no
    public static final int feedBallButtonID = 2;
    public static final int TurnTurretRightButton = 6;
    public static final int TurnTurretLeftButton = 5;
    public static final int TrackTargetButtonID = 1;

  /* These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
     These characterization values MUST be determined either experimentally or
     theoretically
     for *your* robot's drive.*/
  public static final double ks = 0.65147;// Volts
  public static final double kv = 2.6532;// VoltSecondsPerMeter
  public static final double ka = 0.39563; // VoltSecondsSquaredPerMeter

  public static final double WheelBaseWith = 0.80952;

  public static final double MaxSpeed = 3;
  public static final double MaxAcceleration = 1.5;
  //public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // Reasonable baseline values for a RAMSETE follower in units of meters and
  // seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  // PID controller values
  public static final double kp = 3.6702;//125.97
  public static final double ki = 0;
  public static final double kd = 0;//8.9609

  //Limelight
  public static final double goalHeightfeet = 8.5521; //height of the target
  public static final double limelightHeightFeet = 23/12; //height of your camera above the floor 
  public static final double limelightMountAngleDegrees = 33; //mounting angle

  // other constants
  public static final double WheelRadiusInches = 2.125;
  public static final double GearRatio = (30.0/18.0)*5.0;
  
  
    
}
