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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.pathPlanner.com.pathplanner.lib.PathPlanner;
import frc.robot.pathPlanner.com.pathplanner.lib.PathPlannerTrajectory;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  public static AHRS navx;
  
  static Trajectory Auto2;

  static Trajectory Auto4Part1;
  static Trajectory Auto4Part2;
  static Trajectory Auto4Part3;

  static Trajectory Auto5Part1;
  static Trajectory Auto5Part2;
  static Trajectory Auto5Part3;
  static Trajectory Auto5Part4;
  static Trajectory Auto5Part5;

  static Trajectory Auto2StealPart1;
  static Trajectory Auto2StealPart2;
  static Trajectory Auto2StealPart3;
  static Trajectory Auto2StealPart4;
  static Trajectory Auto2StealPart5;

  static PathPlannerTrajectory PP5ballPart1;
  static PathPlannerTrajectory PP5ballPart2;
  static PathPlannerTrajectory PP5ballPart3;
  static PathPlannerTrajectory PP5Ball;

  static Trajectory Test;
  
  
  String Auto2JSON = "paths/Auto2.wpilib.json";

  String Auto4Part1JSON = "paths/Auto4Part1.wpilib.json";
  String Auto4Part2JSON = "paths/Auto4Part2.wpilib.json";
  String Auto4Part3JSON = "paths/Auto4Part3.wpilib.json";

  String Auto5Part1JSON = "paths/Auto5Part1.wpilib.json";
  String Auto5Part2JSON = "paths/Auto5Part2.wpilib.json";
  String Auto5Part3JSON = "paths/Auto5Part3.wpilib.json";
  String Auto5Part4JSON = "paths/Auto5part4.wpilib.json";
  String Auto5Part5JSON = "paths/Auto5part5.wpilib.json";
  
  String Auto2StealPart1JSON = "paths/Auto2StealPart1.wpilib.json";
  String Auto2StealPart2JSON = "paths/Auto2StealPart2.wpilib.json";
  String Auto2StealPart3JSON = "paths/Auto2StealPart3.wpilib.json";
  String Auto2StealPart4JSON = "paths/Auto2StealPart4.wpilib.json";
  String Auto2StealPart5JSON = "paths/Auto2StealPart5.wpilib.json";

  String TestJSON = "paths/Test.wpilib.json";


  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    navx = new AHRS(Port.kUSB1); 
    navx.enableLogging(true);
    //calibrate();
    
    InitTrajectorys();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    resetGyro();
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

    PP5ballPart1 = PathPlanner.loadPath("5ballpart1", Constants.MaxSpeed, Constants.MaxAcceleration);
    PP5ballPart2 = PathPlanner.loadPath("5ballpart2", Constants.MaxSpeed, Constants.MaxAcceleration);
    PP5ballPart3 = PathPlanner.loadPath("5ballpart3", Constants.MaxSpeed, Constants.MaxAcceleration);
    PP5Ball = PathPlanner.loadPath("5Ball", Constants.MaxSpeed, Constants.MaxAcceleration);

    try {
      Path Auto2Path = Filesystem.getDeployDirectory().toPath().resolve(Auto2JSON);
      Auto2 = TrajectoryUtil.fromPathweaverJson(Auto2Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto2JSON, ex.getStackTrace());
    }

    try {
      Path Auto4Part1Path = Filesystem.getDeployDirectory().toPath().resolve(Auto4Part1JSON);
      Auto4Part1 = TrajectoryUtil.fromPathweaverJson(Auto4Part1Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto4Part1JSON, ex.getStackTrace());
    }
     
    try {
      Path Auto4Part2Path = Filesystem.getDeployDirectory().toPath().resolve(Auto4Part2JSON);
      Auto4Part2 = TrajectoryUtil.fromPathweaverJson(Auto4Part2Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto4Part2JSON, ex.getStackTrace());
    }

    try {
      Path Auto4Part3Path = Filesystem.getDeployDirectory().toPath().resolve(Auto4Part3JSON);
      Auto4Part3 = TrajectoryUtil.fromPathweaverJson(Auto4Part3Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto4Part3JSON, ex.getStackTrace());
    }

    try {
      Path Auto5Part1Path = Filesystem.getDeployDirectory().toPath().resolve(Auto5Part1JSON);
      Auto5Part1 = TrajectoryUtil.fromPathweaverJson(Auto5Part1Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto5Part1JSON, ex.getStackTrace());
    }
     
    try {
      Path Auto5Part2Path = Filesystem.getDeployDirectory().toPath().resolve(Auto5Part2JSON);
      Auto5Part2 = TrajectoryUtil.fromPathweaverJson(Auto5Part2Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto5Part2JSON, ex.getStackTrace());
    }

    try {
      Path Auto5Part3Path = Filesystem.getDeployDirectory().toPath().resolve(Auto5Part3JSON);
      Auto5Part3 = TrajectoryUtil.fromPathweaverJson(Auto5Part3Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto5Part3JSON, ex.getStackTrace());
    }

    try {
      Path Auto5Part4Path = Filesystem.getDeployDirectory().toPath().resolve(Auto5Part4JSON);
      Auto5Part4 = TrajectoryUtil.fromPathweaverJson(Auto5Part4Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto5Part4JSON, ex.getStackTrace());
    }

    try {
      Path Auto5Part5Path = Filesystem.getDeployDirectory().toPath().resolve(Auto5Part5JSON);
      Auto5Part5 = TrajectoryUtil.fromPathweaverJson(Auto5Part5Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto5Part5JSON, ex.getStackTrace());
    }

    try {
      Path Auto2StealPart1Path = Filesystem.getDeployDirectory().toPath().resolve(Auto2StealPart1JSON);
      Auto2StealPart1 = TrajectoryUtil.fromPathweaverJson(Auto2StealPart1Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto2StealPart1JSON, ex.getStackTrace());
    }

    try {
      Path Auto2StealPart2Path = Filesystem.getDeployDirectory().toPath().resolve(Auto2StealPart2JSON);
      Auto2StealPart2 = TrajectoryUtil.fromPathweaverJson(Auto2StealPart2Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto2StealPart2JSON, ex.getStackTrace());
    }

    try {
      Path Auto2StealPart3Path = Filesystem.getDeployDirectory().toPath().resolve(Auto2StealPart3JSON);
      Auto2StealPart3 = TrajectoryUtil.fromPathweaverJson(Auto2StealPart3Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto2StealPart3JSON, ex.getStackTrace());
    }

    try {
      Path Auto2StealPart4Path = Filesystem.getDeployDirectory().toPath().resolve(Auto2StealPart4JSON);
      Auto2StealPart4 = TrajectoryUtil.fromPathweaverJson(Auto2StealPart4Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto2StealPart4JSON, ex.getStackTrace());
    }

    try {
      Path Auto2StealPart5Path = Filesystem.getDeployDirectory().toPath().resolve(Auto2StealPart5JSON);
      Auto2StealPart5 = TrajectoryUtil.fromPathweaverJson(Auto2StealPart5Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Auto2StealPart5JSON, ex.getStackTrace());
    }

    try {
      Path TestPath = Filesystem.getDeployDirectory().toPath().resolve(TestJSON);
      Test = TrajectoryUtil.fromPathweaverJson(TestPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + TestJSON, ex.getStackTrace());
    }

  }

  public static Trajectory getAuto2Trajectory() {
    return Auto2;
  }

  public static Trajectory getAuto4Part1Trajectory() {
    return Auto4Part1;
  }

  public static Trajectory getAuto4Part2Trajectory() {
    return Auto4Part2;
  }

  public static Trajectory getAuto4Part3Trajectory() {
    return Auto4Part3;
  }

  public static Trajectory getAuto5Part1Trajectory() {
    return Auto5Part1;
  }

  public static Trajectory getAuto5Part2Trajectory() {
    return Auto5Part2;
  }

  public static Trajectory getAuto5Part3Trajectory() {
    return Auto5Part3;
  }

  public static Trajectory getAuto5Part4Trajectory() {
    return Auto5Part4;
  }

  public static Trajectory getAuto5Part5Trajectory() {
    return Auto5Part5;
  }

  public static Trajectory getAuto2StealPart1Trajectory() {
    return Auto2StealPart1;
  }

  public static Trajectory getAuto2StealPart2Trajectory() {
    return Auto2StealPart2;
  }

  public static Trajectory getAuto2StealPart3Trajectory() {
    return Auto2StealPart3;
  }

  public static Trajectory getAuto2StealPart4Trajectory() {
    return Auto2StealPart4;
  }

  public static Trajectory getAuto2StealPart5Trajectory() {
    return Auto2StealPart5;
  }

  public static Trajectory getTestTrajectory() {
    return Test;
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
    SmartDashboard.putNumber("angle", getHeading().getDegrees());
  }

  
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    robotContainer.shooter.setLEDs(0, 255, 0);
    if(DriverStation.isFMSAttached()) Shuffleboard.stopRecording();
    
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if(DriverStation.isFMSAttached()) Shuffleboard.startRecording();
    if(DriverStation.getAlliance() == Alliance.Red){
      robotContainer.shooter.setLEDs(255, 0, 0);
    }else if(DriverStation.getAlliance() == Alliance.Blue){
      robotContainer.shooter.setLEDs(0, 0, 255);
    }
    
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousExit() {
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    robotContainer.shooter.setLEDs(0, 18, 222);
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
    if(robotContainer.shooter.isMainSpeedingUp && robotContainer.shooter.isMainUpToSpeed && robotContainer.camera.v == 1 && robotContainer.shooter.isTopSpeedingUp && robotContainer.shooter.isTopUpToSpeed){
      robotContainer.shooter.setLEDs(0, 255, 0); 
    }else if(DriverStation.getMatchTime() < 20){
      robotContainer.shooter.setLEDs(235, 102, 30);
    }else if(DriverStation.getMatchTime() < 30){
      robotContainer.shooter.setLEDs(145, 5, 156);
    }else if(DriverStation.getAlliance() == Alliance.Red){
      robotContainer.shooter.setLEDs(255, 0, 0);
    }else if(DriverStation.getAlliance() == Alliance.Blue){
      robotContainer.shooter.setLEDs(0, 0, 255);
    }
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
