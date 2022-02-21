// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  public static AHRS navx = new AHRS();
  static Trajectory Auto1Part1 = new Trajectory();
  static Trajectory Auto1Part2 = new Trajectory();
  static Trajectory Auto2Part1 = new Trajectory();
  static Trajectory Auto2Part2 = new Trajectory();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    calibrate();
    resetGyro();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    InitTrajectorys();
  }
  
  public void resetGyro() {
    navx.reset();
  }

  public void calibrate() {
    navx.calibrate();
  }

  public static Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navx.getAngle());
  }
  public static Rotation2d getRotation2d(){
    return navx.getRotation2d();
  }

  public void InitTrajectorys() {
    String Auto1Part1JSON = "paths/Auto1Part1.wpilib.json";
    String Auto1Part2JSON = "paths/Auto1Part2.wpilib.json";
    String Auto2Part1JSON = "paths/Auto2Part1.wpilib.json";
    String Auto2Part2JSON = "paths/Auto2Part2.wpilib.json";
    try {
      Path Auto1Part1Path = Filesystem.getDeployDirectory().toPath().resolve(Auto1Part1JSON);
      Auto1Part1 = TrajectoryUtil.fromPathweaverJson(Auto1Part1Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto1Part1JSON, ex.getStackTrace());
    }
     
    try {
      Path Auto1Part2Path = Filesystem.getDeployDirectory().toPath().resolve(Auto1Part2JSON);
      Auto1Part2 = TrajectoryUtil.fromPathweaverJson(Auto1Part2Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto1Part2JSON, ex.getStackTrace());
    }

    try {
      Path Auto2Part1Path = Filesystem.getDeployDirectory().toPath().resolve(Auto2Part1JSON);
      Auto2Part1 = TrajectoryUtil.fromPathweaverJson(Auto2Part1Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto2Part1JSON, ex.getStackTrace());
    }
     
    try {
      Path Auto2Part2Path = Filesystem.getDeployDirectory().toPath().resolve(Auto2Part2JSON);
      Auto2Part2 = TrajectoryUtil.fromPathweaverJson(Auto2Part2Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto2Part2JSON, ex.getStackTrace());
    }
  }

  public static Trajectory getAuto1Part1Trajectory() {
    return Auto1Part1;
  }

  public static Trajectory getAuto1Part2Trajectory() {
    return Auto1Part2;
  }

  public static Trajectory getAuto2Part1Trajectory() {
    return Auto2Part1;
  }

  public static Trajectory getAuto2Part2Trajectory() {
    return Auto2Part2;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
