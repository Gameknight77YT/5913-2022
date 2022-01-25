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
    public static final int climberMotorID = 7;
    public static final int climberSlaveID = 8;
    public static final int intakeMotorID = 9;
    public static final int feederMotorID = 10;
    public static final int pcmID = 11;

    //solinoid ids 
    public static final int swingForwardID = 0;
    public static final int swingReverseID = 1;
    public static final int intakeArmsBackwardID = 2;
    public static final int intakeArmsForwardID = 3;

    //joystick ids
    public static final int driverjoystickID = 0;
    public static final int manipulatorJoystickID = 1;
    public static final int joystickX = 0;
    public static final int joystickY = 1;

    //speed  constants
    public static final double speedX = .5;
    public static final double speedY = .5;
    public static final double climberSpeed = .90;
    public static final double ShooterSpeed1 = 4000;// 7 
    public static final double TopShooterSpeed1 = 20000;// 7 
    public static final double ShooterSpeed2 = 4000;// 9 
    public static final double TopShooterSpeed2 = 15000;// 9 
    public static final double ShooterSpeed3 = 15000;// 11 
    public static final double TopShooterSpeed3 = 9000;// 11 
    public static final double climbEncoderTop = 10000;
    public static final double intakeSpeed = .65;
    public static final double feederSpeed = .55;

    //buttons
    public static final int climbUpButtonID = 5;
    public static final int climbDownButtonID = 3;
    public static final int swingInButtonID = 6;
    public static final int swingOutButtonID = 4;
    public static final int shootBall3ButtonID = 11;
    public static final int shootBall2ButtonID = 9;
    public static final int shootBall1ButtonID = 7;

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

  // other constants
  public static final double WheelRadiusInches = 3;
  public static final double GearRatio = 14.014;
    
}
