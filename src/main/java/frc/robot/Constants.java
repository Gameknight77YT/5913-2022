// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int pcmID = 10;
    public static final int climberMotorID = 0;

    public static final int swingForwardID = 0;
    public static final int swingReverseID = 0;

    //joystick ids
    public static final int driverjoystickID = 0;
    public static final int manipulatorJoystickID = 1;
    public static final int driverjoystickX = 0;
    public static final int driverjoystickY = 1;

    //speed  constants
    public static final double speedX = .5;
    public static final double speedY = .5;
    public static final double climberSpeed = .95;
    public static final double ShooterSpeed1 = 20900;// 7
    public static final double ShooterSpeed2 = 22000;// 9
    public static final double ShooterSpeed3 = 23500;// 11
    public static final double climbEncoderTop = 0;

    //buttons
    public static final int climbUpButtonID = 0;
    public static final int climbDownButtonID = 0;
    public static final int swingInButtonID = 0;
    public static final int swingOutButtonID = 0;
    public static final int shootBall3ButtonID = 11;
    public static final int shootBall2ButtonID = 9;
    public static final int shootBall1ButtonID = 11;

    //pid and other
    public static final double WheelBaseWith = 3;
    public static final double ks = 0;
    public static final double kv = 0;
    public static final double ka = 0;
    public static final double kp = 0;
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double kRamseteB = 0;
    public static final double kRamseteZeta = 0;
    public static final double GearRatio = 1;
    public static final double WheelRadiusInches = 0;
    
}
