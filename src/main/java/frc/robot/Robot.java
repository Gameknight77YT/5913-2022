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
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  //static Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  //static AHRS navx = new AHRS(SerialPort.Port.kUSB);
  static Trajectory TestTrajectory = new Trajectory();
  static Trajectory GameDefault = new Trajectory();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //calibrate();
    //resetGyro();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //InitTrajectorys();
  }
  
  /*public void resetGyro() {
    //gyro.reset();
    navx.reset();
  }

  public void calibrate() {
    //gyro.calibrate();
    navx.calibrate();
  }

  public static Rotation2d getHeading() {
    //return Rotation2d.fromDegrees(-gyro.getAngle());
    return Rotation2d.fromDegrees(-navx.getAngle());
  }
  public static Rotation2d getRotation2d(){
    //return gyro.getRotation2d();
    return navx.getRotation2d();
  }

  public void InitTrajectorys() {
    try {
      Path TesttrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Test.wpilib.json");
      TestTrajectory = TrajectoryUtil.fromPathweaverJson(TesttrajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + "output/Test.wpilib.json", ex.getStackTrace());
    }
       String GameDefaultJSON = "output/GameDefault.wpilib.json";
     try {
      Path GameDefaultPath = Filesystem.getDeployDirectory().toPath().resolve(GameDefaultJSON);
      GameDefault = TrajectoryUtil.fromPathweaverJson(GameDefaultPath);
     } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + GameDefaultJSON, ex.getStackTrace());
     }
  }

  

  public static Trajectory getTestTrajectory() {
    return TestTrajectory;
  }
  public static Trajectory getGameDefaultTrajectory() {
    return GameDefault;
  }*/

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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
