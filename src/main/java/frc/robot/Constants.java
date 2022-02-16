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
    public static final int intakeSystemMotorID = 11;
    public static final int TurretControlID = 12;
    public static final int pcmID = 13;
    public static final int pdpID = 14;

    //solinoid ids 
    public static final int swingForwardID = 2;
    public static final int swingReverseID = 3;
    public static final int intakeArmsBackwardID = 1;
    public static final int intakeArmsForwardID = 0;

    //joystick ids
    public static final int driverjoystickID = 0;
    public static final int manipulatorJoystickID = 1;
    public static final int joystickX = 0;
    public static final int joystickY = 1;
    public static final int JoystickZAxisID = 2;

    //speed  constants
    public static final double speedX = .95;
    public static final double speedY = .95;
    public static final double TurnTurretSpeed = .95;
    public static final double climberSpeed = 1;
    public static final double ShooterSpeed1 = 1500;// 7 
    public static final double TopShooterSpeed1 = 7500;// 7 
    public static final double ShooterSpeed2 = 7250;// 9 
    public static final double TopShooterSpeed2 = 7000;// 9 
    public static final double ShooterSpeed3 = 8000;// 11 
    public static final double TopShooterSpeed3 = 14000;// 11 
    public static final double ShooterSpeed4 = 7000;// 10 
    public static final double TopShooterSpeed4 = 20000;// 10
    public static final double intakeSpeed = .95;
    public static final double feederSpeed = .95;
    public static final double intakeSystemSpeed = .95;
    public static final double autoSpeed = .95;

    //times / encoder counts
    public static final double stopAndShootTime = 2;
    public static final double backUpDistance = 20000;
    public static final double climbEncoderTop = 207631;

    //buttons
    public static final int climbUpButtonID = 3;
    public static final int climbDownButtonID = 4;
    public static final int swingInButtonID = 3;
    public static final int swingOutButtonID = 4;
    public static final int shootBallAutoSpeedButtonID = 12;
    public static final int shootBall4ButtonID = 10;
    public static final int shootBall3ButtonID = 11;
    public static final int shootBall2ButtonID = 9;
    public static final int shootBall1ButtonID = 7;
    public static final int toggleIntakeArmsButtonID = 2; 
    public static final int intakeBallButtonID = 3;
    public static final int outTakeBallButtonID = 8;
    public static final int feedBallButtonID = 5;
    public static final int ActivateTurnTurretButton = 4;
    public static final int TrackTargetButtonID = 1;

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or
  // theoretically
  // for *your* robot's drive.
  public static final double ks = 0.706;// Volts
  public static final double kv = 2.1;// VoltSecondsPerMeter
  public static final double ka = 0.372; // VoltSecondsSquaredPerMeter

  public static final double WheelBaseWith = Units.inchesToMeters(25);

  public static final double MaxSpeedMetersPerSecond = 1;
  public static final double MaxAccelerationMetersPerSecondSquared = 3;
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // Reasonable baseline values for a RAMSETE follower in units of meters and
  // seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  // PID controller values
  public static final double kp = 2.81;
  public static final double ki = 0;
  public static final double kd = 0;

  //Limelight TODO:change these
  public static final double h2 = 0; //height of the target
  public static final double h1 = 0; //height of your camera above the floor 
  public static final double a1 = 0; //mounting angle

  // other constants
  public static final double WheelRadiusInches = 2.125;
  public static final double GearRatio = 7;
    
}
