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
    public static final int shooterID = 5;
    public static final int climberMotorID = 6;
    public static final int intakeMotorID = 7;
    public static final int feederMotorID = 8;
    public static final int pcmID = 10;

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
    public static final double climberSpeed = .95;
    public static final double ShooterSpeed1 = 20900;// 7 20900
    public static final double ShooterSpeed2 = 22000;// 9 2200
    public static final double ShooterSpeed3 = 23500;// 11 23500
    public static final double climbEncoderTop = 0;

    //buttons
    public static final int climbUpButtonID = 10;
    public static final int climbDownButtonID = 12;
    public static final int swingInButtonID = 10;
    public static final int swingOutButtonID = 12;
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

  public static final double kMaxSpeedMetersPerSecond = 1;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

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
public static final double intakeSpeed = 0;
public static final double feederSpeed = 0;
    
}
