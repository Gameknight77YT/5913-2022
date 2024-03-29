// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.pathPlanner.com.pathplanner.lib.commands.PPRamseteCommand;
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
  public Shooter shooter;
  public Camera camera;
  private Intake intake;
  private ClimbArms climbArms;

  private DriveWithJoysticks driveWithJoysticks; 
  private ControlClimber controlClimber;
  private SwingIn swingIn;
  private SwingOut swingOut;
  private HookIn hookIn;
  private HookOut hookOut;
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
  private OutTakeBall2 outTakeBall2;
  private FeedBall feedBall;
  private TrackTarget trackTarget;
  private ControlTurret controlTurret;

  private SendableChooser<Integer> autoChooser;

  private Joystick driverJoystick;
  private Joystick manipulatorJoystick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverJoystick = new Joystick(Constants.driverJoystickID);
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
    hookIn = new HookIn(climbArms);
    hookOut = new HookOut(climbArms);
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
    outTakeBall2 = new OutTakeBall2(intake);
    feedBall = new FeedBall(intake);
    trackTarget = new TrackTarget(camera);
    controlTurret = new ControlTurret(camera, manipulatorJoystick);
    camera.setDefaultCommand(controlTurret);

    autoChooser = new SendableChooser<Integer>();
    autoChooser.addOption("auto2", 2);
    autoChooser.addOption("auto2steal", 3);
    autoChooser.addOption("auto4", 4);
    autoChooser.setDefaultOption("auto5", 5);
    autoChooser.addOption("test", 1);
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

    JoystickButton hookInButton = new JoystickButton(manipulatorJoystick, Constants.hooksInButtonID);
    hookInButton.whileHeld(hookIn);

    JoystickButton hookOutButton = new JoystickButton(manipulatorJoystick, Constants.hooksOutButtonID);
    hookOutButton.whileHeld(hookOut);

    //JoystickButton shootBall1Button = new JoystickButton(manipulatorJoystick, Constants.shootBall1ButtonID);
    //shootBall1Button.whileHeld(shootBall1);

    JoystickButton shootBall2Button = new JoystickButton(manipulatorJoystick, Constants.shootBall2ButtonID);
    shootBall2Button.whileHeld(shootBall2);

    JoystickButton shootBall3Button = new JoystickButton(manipulatorJoystick, Constants.shootBall3ButtonID);
    shootBall3Button.whileHeld(shootBall3);

    JoystickButton shootBall4Button = new JoystickButton(manipulatorJoystick, Constants.shootBall4ButtonID);
    shootBall4Button.whileHeld(shootBall4);

    //JoystickButton shootBall6Button = new JoystickButton(manipulatorJoystick, Constants.shootBall6ButtonID);
    //shootBall6Button.whileHeld(shootBall6);

    JoystickButton shootBallAutoSpeedButton = new JoystickButton(manipulatorJoystick, Constants.shootBallAutoSpeedButtonID);
    shootBallAutoSpeedButton.whileHeld(shootBallAutoSpeed);

    JoystickButton feedBallButton = new JoystickButton(manipulatorJoystick, Constants.feedBallButtonID);
    feedBallButton.whileHeld(feedBall);

    JoystickButton TrackTargetButton = new JoystickButton(manipulatorJoystick, Constants.TrackTargetButtonID);
    TrackTargetButton.whileHeld(trackTarget);

    JoystickButton intakeArmsUpButton = new JoystickButton(driverJoystick, Constants.intakeArmsOutButtonID);
    intakeArmsUpButton.whenPressed(intakeArmsUp);
    intakeArmsUpButton.whenHeld(intakeBall);

    JoystickButton intakeArmsDownButton = new JoystickButton(driverJoystick, Constants.intakeArmsInButtonID);
    intakeArmsDownButton.whenPressed(intakeArmsDown);

    JoystickButton intakeBallButton = new JoystickButton(driverJoystick, Constants.intakeBallButtonID);
    intakeBallButton.whileHeld(intakeBall);

    JoystickButton outTakeBallButton = new JoystickButton(driverJoystick, Constants.outTakeBallButtonID);
    outTakeBallButton.whileHeld(outTakeBall);

    JoystickButton outTakeBall2Button = new JoystickButton(driverJoystick, Constants.outTakeBall2ButtonID);
    outTakeBall2Button.whileHeld(outTakeBall2);
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
      return new TurnToAngle(driveTrain, 45)//Test1command
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

      return new HookAndSwingOut(climbArms)
      .andThen((Auto2command.raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(driveTrain, shooter, camera, intake, .2, 3))
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

      return new HookAndSwingOut(climbArms)
      .andThen(Auto4Part1command.raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(driveTrain, shooter, camera, intake, 0, .6))
      .andThen(Auto4Part2command.raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new WaitCommand(1).raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(Auto4Part3command.raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(driveTrain, shooter, camera, intake, .5, 2.5))
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

      return new HookAndSwingOut(climbArms)
      .andThen(Auto5Part1command.raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(driveTrain, shooter, camera, intake, 0, .6))
      .andThen(Auto5Part2command.raceWith(new AutoIntake(camera, shooter, intake, false)))
      .andThen(Auto5Part3command.raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(driveTrain, shooter, camera, intake, 0, .7))
      .andThen(Auto5Part4command.raceWith(new AutoIntake(camera, shooter, intake, false)))
      .andThen(new WaitCommand(.25).raceWith(new AutoIntake(camera, shooter, intake, false)))
      .andThen(Auto5Part5command.raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(driveTrain, shooter, camera, intake, 0, 1))
      ;

      /*HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("one", new InstantCommand());

      PPRamseteCommand auto5Command = new PPRamseteCommand(
        Robot.PP5Ball, 
        driveTrain::getPose, 
        new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta), 
        driveTrain.getFeedForward(), 
        driveTrain.getKinematics(), 
        driveTrain::getSpeeds, 
        driveTrain.getleftPidController(), 
        driveTrain.getrightPidController(), 
        driveTrain::tankDriveVolts, 
        eventMap,
        driveTrain
        );*/

      /*PPRamseteCommand Auto5Part1command = new PPRamseteCommand(
        Robot.PP5ballPart1, 
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

      PPRamseteCommand Auto5Part2command = new PPRamseteCommand(
        Robot.PP5ballPart2, 
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

      PPRamseteCommand Auto5Part3command = new PPRamseteCommand(
        Robot.PP5ballPart3, 
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

      return new HookAndSwingOut(climbArms)
      .andThen(Auto5Part1command.raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(driveTrain, shooter, camera, intake, 0, .6))
      .andThen(Auto5Part2command.raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(driveTrain, shooter, camera, intake, 0, 1))
      .andThen(Auto5Part3command.raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(driveTrain, shooter, camera, intake, 0, .7))
      ;*/


    }else if(autoChooser.getSelected() == 3){

      RamseteCommand Auto2StealPart1command = new RamseteCommand(
        Robot.getAuto2StealPart1Trajectory(), 
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
      
      RamseteCommand Auto2StealPart2command = new RamseteCommand(
        Robot.getAuto2StealPart2Trajectory(), 
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
      
      RamseteCommand Auto2StealPart3command = new RamseteCommand(
        Robot.getAuto2StealPart3Trajectory(), 
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

        RamseteCommand Auto2StealPart4command = new RamseteCommand(
        Robot.getAuto2StealPart4Trajectory(), 
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

        RamseteCommand Auto2StealPart5command = new RamseteCommand(
        Robot.getAuto2StealPart5Trajectory(), 
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

        driveTrain.resetOdometry(Robot.getAuto2StealPart1Trajectory().getInitialPose());

      return new HookAndSwingOut(climbArms)//-105, -120, 
      .andThen(Auto2StealPart1command.raceWith(new AutoIntake(camera, shooter, intake, true)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new StopAndShoot(driveTrain, shooter, camera, intake, 0, 1))
      .andThen(new TurnToAngle(driveTrain, -65))
      .andThen(() -> driveTrain.resetOdometry(Robot.getAuto2StealPart2Trajectory().getInitialPose()))
      .andThen(Auto2StealPart2command.raceWith(new AutoIntakeNoShoot(intake)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new TurnToAngle(driveTrain, -170))
      .andThen(() -> driveTrain.resetOdometry(Robot.getAuto2StealPart3Trajectory().getInitialPose()))
      .andThen(Auto2StealPart3command.raceWith(new AutoIntakeNoShoot(intake)))
      .andThen(Auto2StealPart4command.raceWith(new IntakeArmsDown(intake)))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new TurnToAngle(driveTrain, 70))
      .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
      .andThen(new OutTakeBall2(intake).raceWith(new WaitCommand(2)))
      ;

    }else{
      return null;
    }
    
  }
}
