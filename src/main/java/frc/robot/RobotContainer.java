// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveTrain driveTrain;
  private Climber climber;
  private Shooter shooter;
  private Camera camera;
  private Intake intake;
  private ClimbArms climbArms;

  private DriveWithJoysticks driveWithJoysticks; 
  private ControlClimber controlClimber;
  private SwingIn swingIn;
  private SwingOut swingOut;
  private Shootball1 shootBall1;
  private Shootball2 shootBall2;
  private Shootball3 shootBall3;
  private Shootball4 shootBall4;
  private Shootball6 shootBall6;
  private ShootBallAutoSpeed shootBallAutoSpeed;
  private IntakeArmsUp intakeArmsUp;
  private IntakeArmsDown intakeArmsDown;
  private IntakeBall intakeBall;
  private OutTakeBall outTakeBall;
  private FeedBall feedBall;
  private TrackTarget trackTarget;
  private ControlTurret controlTurret;
  private AutoIntake autoIntake;
  private StopAndShoot stopAndShoot;

  private SendableChooser<Integer> autoChooser;

  private Joystick driverJoystick;
  private Joystick manipulatorJoystick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverJoystick = new Joystick(Constants.driverjoystickID);
    manipulatorJoystick = new Joystick(Constants.manipulatorJoystickID);

    driveTrain = new DriveTrain();
    climber = new Climber();
    shooter = new Shooter();
    camera = new Camera();
    intake = new Intake();
    climbArms = new ClimbArms();

    driveWithJoysticks = new DriveWithJoysticks(driveTrain, driverJoystick);
    driveTrain.setDefaultCommand(driveWithJoysticks);
    swingIn = new SwingIn(climbArms);
    swingOut = new SwingOut(climbArms);
    controlClimber = new ControlClimber(climber, driverJoystick);
    climber.setDefaultCommand(controlClimber);
    shootBall1 = new Shootball1(shooter);
    shootBall2 = new Shootball2(shooter);
    shootBall3 = new Shootball3(shooter);
    shootBall4 = new Shootball4(shooter);
    shootBall6 = new Shootball6(shooter);
    shootBallAutoSpeed = new ShootBallAutoSpeed(shooter);
    intakeArmsUp = new IntakeArmsUp(intake);
    intakeArmsDown = new IntakeArmsDown(intake);
    intakeBall = new IntakeBall(intake);
    outTakeBall = new OutTakeBall(intake);
    feedBall = new FeedBall(intake);
    trackTarget = new TrackTarget(camera);
    controlTurret = new ControlTurret(camera, manipulatorJoystick);
    camera.setDefaultCommand(controlTurret);
    autoIntake = new AutoIntake(camera, shooter, intake);
    stopAndShoot = new StopAndShoot(shooter, camera, intake);

    autoChooser = new SendableChooser<Integer>();
    autoChooser.addOption("auto2", 2);
    autoChooser.addOption("auto4", 4);
    autoChooser.setDefaultOption("auto5", 5);
    autoChooser.addOption("Test", 1);
    SmartDashboard.putData(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton swingInButton = new JoystickButton(manipulatorJoystick, Constants.swingInButtonID);
    swingInButton.whileHeld(swingIn);

    JoystickButton swingOutButton = new JoystickButton(manipulatorJoystick, Constants.swingOutButtonID);
    swingOutButton.whileHeld(swingOut);

    JoystickButton shootBall1Button = new JoystickButton(manipulatorJoystick, Constants.shootBall1ButtonID);
    shootBall1Button.whileHeld(shootBall1);

    //JoystickButton shootBall2Button = new JoystickButton(manipulatorJoystick, Constants.shootBall2ButtonID);
    //shootBall2Button.whileHeld(shootBall2);

    JoystickButton shootBall3Button = new JoystickButton(manipulatorJoystick, Constants.shootBall3ButtonID);
    shootBall3Button.whileHeld(shootBall3);

    JoystickButton shootBall4Button = new JoystickButton(manipulatorJoystick, Constants.shootBall4ButtonID);
    shootBall4Button.whileHeld(shootBall4);

    JoystickButton shootBall6Button = new JoystickButton(manipulatorJoystick, Constants.shootBall6ButtonID);
    shootBall6Button.whileHeld(shootBall6);

    JoystickButton shootBallAutoSpeedButton = new JoystickButton(manipulatorJoystick, Constants.shootBallAutoSpeedButtonID);
    shootBallAutoSpeedButton.whileHeld(shootBallAutoSpeed);

    JoystickButton intakeArmsUpButton = new JoystickButton(driverJoystick, Constants.intakeArmsUpButtonID);
    intakeArmsUpButton.whenPressed(intakeArmsUp);

    JoystickButton intakeArmsDownButton = new JoystickButton(driverJoystick, Constants.intakeArmsDownButtonID);
    intakeArmsDownButton.whenPressed(intakeArmsDown);

    JoystickButton intakeBallButton = new JoystickButton(manipulatorJoystick, Constants.intakeBallButtonID);
    intakeBallButton.whileHeld(intakeBall);

    JoystickButton outTakeBallButton = new JoystickButton(manipulatorJoystick, Constants.outTakeBallButtonID);
    outTakeBallButton.whileHeld(outTakeBall);

    JoystickButton feedBallButton = new JoystickButton(manipulatorJoystick, Constants.feedBallButtonID);
    feedBallButton.whileHeld(feedBall);

    JoystickButton TrackTargetButton = new JoystickButton(manipulatorJoystick, Constants.TrackTargetButtonID);
    TrackTargetButton.whileHeld(trackTarget);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if(autoChooser.getSelected() == 1){
      RamseteCommand Test1command = new RamseteCommand(
        Robot.getTestTrajectory(), 
        driveTrain::getPose,
        new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
        driveTrain.getFeedForward(),
        driveTrain.getKinematics(),
        driveTrain::getSpeeds,
        driveTrain.getleftPidController(),
        driveTrain.getrightPidController(),
        driveTrain::tankDriveVolts,
        driveTrain
        );

        driveTrain.resetOdometry(Robot.getTestTrajectory().getInitialPose());
        
      return Test1command
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      ;

    }else if(autoChooser.getSelected() == 2){
      RamseteCommand Auto2command = new RamseteCommand(
        Robot.getAuto2Trajectory(), 
        driveTrain::getPose,
        new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
        driveTrain.getFeedForward(),
        driveTrain.getKinematics(),
        driveTrain::getSpeeds,
        driveTrain.getleftPidController(),
        driveTrain.getrightPidController(),
        driveTrain::tankDriveVolts,
        driveTrain
        );
      
        

      driveTrain.resetOdometry(Robot.getAuto2Trajectory().getInitialPose());

      return ((Auto2command.raceWith(new AutoIntake(camera, shooter, intake)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(shooter, camera, intake))
      );

    
    }else if(autoChooser.getSelected() == 4){
      RamseteCommand Auto4Part1command = new RamseteCommand(
        Robot.getAuto4Part1Trajectory(), 
        driveTrain::getPose,
        new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
        driveTrain.getFeedForward(),
        driveTrain.getKinematics(),
        driveTrain::getSpeeds,
        driveTrain.getleftPidController(),
        driveTrain.getrightPidController(),
        driveTrain::tankDriveVolts,
        driveTrain
        );
      
        RamseteCommand Auto4Part2command = new RamseteCommand(
        Robot.getAuto4Part2Trajectory(), 
        driveTrain::getPose,
        new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
        driveTrain.getFeedForward(),
        driveTrain.getKinematics(),
        driveTrain::getSpeeds,
        driveTrain.getleftPidController(),
        driveTrain.getrightPidController(),
        driveTrain::tankDriveVolts,
        driveTrain
        );

        RamseteCommand Auto4Part3command = new RamseteCommand(
          Robot.getAuto4Part3Trajectory(), 
          driveTrain::getPose,
          new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
          driveTrain.getFeedForward(),
          driveTrain.getKinematics(),
          driveTrain::getSpeeds,
          driveTrain.getleftPidController(),
          driveTrain.getrightPidController(),
          driveTrain::tankDriveVolts,
          driveTrain
          );

      driveTrain.resetOdometry(Robot.getAuto4Part1Trajectory().getInitialPose());

      return (Auto4Part1command.raceWith(new AutoIntake(camera, shooter, intake)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(shooter, camera, intake))
      .andThen(Auto4Part2command.raceWith(new AutoIntake(camera, shooter, intake)))
      .andThen(Auto4Part3command.raceWith(new AutoIntake(camera, shooter, intake)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(shooter, camera, intake))
      ;

    }else if(autoChooser.getSelected() == 5){
      RamseteCommand Auto5Part1command = new RamseteCommand(
        Robot.getAuto5Part1Trajectory(), 
        driveTrain::getPose,
        new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
        driveTrain.getFeedForward(),
        driveTrain.getKinematics(),
        driveTrain::getSpeeds,
        driveTrain.getleftPidController(),
        driveTrain.getrightPidController(),
        driveTrain::tankDriveVolts,
        driveTrain
        );
      
        RamseteCommand Auto5Part2command = new RamseteCommand(
        Robot.getAuto5Part2Trajectory(), 
        driveTrain::getPose,
        new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
        driveTrain.getFeedForward(),
        driveTrain.getKinematics(),
        driveTrain::getSpeeds,
        driveTrain.getleftPidController(),
        driveTrain.getrightPidController(),
        driveTrain::tankDriveVolts,
        driveTrain
        );

        RamseteCommand Auto5Part3command = new RamseteCommand(
          Robot.getAuto5Part3Trajectory(), 
          driveTrain::getPose,
          new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
          driveTrain.getFeedForward(),
          driveTrain.getKinematics(),
          driveTrain::getSpeeds,
          driveTrain.getleftPidController(),
          driveTrain.getrightPidController(),
          driveTrain::tankDriveVolts,
          driveTrain
          );

        RamseteCommand Auto5Part4command = new RamseteCommand(
          Robot.getAuto5Part4Trajectory(), 
          driveTrain::getPose,
          new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
          driveTrain.getFeedForward(),
          driveTrain.getKinematics(),
          driveTrain::getSpeeds,
          driveTrain.getleftPidController(),
          driveTrain.getrightPidController(),
          driveTrain::tankDriveVolts,
          driveTrain
          );

        RamseteCommand Auto5Part5command = new RamseteCommand(
          Robot.getAuto5Part5Trajectory(), 
          driveTrain::getPose,
          new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
          driveTrain.getFeedForward(),
          driveTrain.getKinematics(),
          driveTrain::getSpeeds,
          driveTrain.getleftPidController(),
          driveTrain.getrightPidController(),
          driveTrain::tankDriveVolts,
          driveTrain
          );

      driveTrain.resetOdometry(Robot.getAuto5Part1Trajectory().getInitialPose());

      return (Auto5Part1command.raceWith(new AutoIntake(camera, shooter, intake)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(shooter, camera, intake))
      .andThen(Auto5Part2command.raceWith(new AutoIntake(camera, shooter, intake)))
      .andThen(Auto5Part3command.raceWith(new AutoIntake(camera, shooter, intake)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(shooter, camera, intake))
      .andThen(Auto5Part4command.raceWith(new AutoIntake(camera, shooter, intake)))
      .andThen(Auto5Part5command.raceWith(new AutoIntake(camera, shooter, intake)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(shooter, camera, intake))
      ;
    }else{
      return null;
    }
    
  }
}
