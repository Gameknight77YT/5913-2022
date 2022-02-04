// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

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

  private DriveWithJoysticks driveWithJoysticks; 
  private ClimbUp climbUp;
  private ClimbDown climbDown;
  private SwingIn swingIn;
  private SwingOut swingOut;
  private Shootball1 shootBall1;
  private Shootball2 shootBall2;
  private Shootball3 shootBall3;
  private ToggleIntakeArms toggleIntakeArms;
  private IntakeBall intakeBall;
  private OutTakeBall outTakeBall;
  private FeedBall feedBall;
  private TrackTarget trackTarget;
  private RamseteCommand AutoPart1command;
  private RamseteCommand AutoPart2command;
  SequentialCommandGroup auto;
  private AutoIntake autoIntake;
  private StopAndShoot stopAndShoot;
  private BackUp backUp;

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

    driveWithJoysticks = new DriveWithJoysticks(driveTrain, driverJoystick);
    driveTrain.setDefaultCommand(driveWithJoysticks);
    climbUp = new ClimbUp(climber);
    climbDown = new ClimbDown(climber);
    swingIn = new SwingIn(climber);
    swingOut = new SwingOut(climber);
    shootBall1 = new Shootball1(shooter);
    shootBall2 = new Shootball2(shooter);
    shootBall3 = new Shootball3(shooter);
    toggleIntakeArms = new ToggleIntakeArms(shooter);
    intakeBall = new IntakeBall(shooter);
    outTakeBall = new OutTakeBall(shooter);
    feedBall = new FeedBall(shooter);
    trackTarget = new TrackTarget(camera);
    autoIntake = new AutoIntake(camera, shooter);
    stopAndShoot = new StopAndShoot(shooter, camera);
    backUp = new BackUp(driveTrain);


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
    JoystickButton climbUpButton = new JoystickButton(driverJoystick, Constants.climbUpButtonID);
    climbUpButton.whileHeld(climbUp);

    JoystickButton climbDownButton = new JoystickButton(driverJoystick, Constants.climbDownButtonID);
    climbDownButton.whileHeld(climbDown);

    JoystickButton swingInButton = new JoystickButton(manipulatorJoystick, Constants.swingInButtonID);
    swingInButton.whileHeld(swingIn);

    JoystickButton swingOutButton = new JoystickButton(manipulatorJoystick, Constants.swingOutButtonID);
    swingOutButton.whileHeld(swingOut);

    JoystickButton shootBall1Button = new JoystickButton(manipulatorJoystick, Constants.shootBall1ButtonID);
    shootBall1Button.whileHeld(shootBall1);

    JoystickButton shootBall2Button = new JoystickButton(manipulatorJoystick, Constants.shootBall2ButtonID);
    shootBall2Button.whileHeld(shootBall2);

    JoystickButton shootBall3Button = new JoystickButton(manipulatorJoystick, Constants.shootBall3ButtonID);
    shootBall3Button.whileHeld(shootBall3);

    JoystickButton toggleIntakeArmsButton = new JoystickButton(manipulatorJoystick, Constants.toggleIntakeArmsButtonID);
    toggleIntakeArmsButton.whenPressed(toggleIntakeArms);

    JoystickButton intakeBallButton = new JoystickButton(manipulatorJoystick, Constants.intakeBallButtonID);
    intakeBallButton.whenPressed(intakeBall);

    JoystickButton outTakeBallButton = new JoystickButton(manipulatorJoystick, Constants.outTakeBallButtonID);
    outTakeBallButton.whenPressed(outTakeBall);

    JoystickButton feedBallButton = new JoystickButton(manipulatorJoystick, Constants.feedBallButtonID);
    feedBallButton.whenPressed(feedBall);

    JoystickButton TrackTargetButton = new JoystickButton(manipulatorJoystick, Constants.TrackTargetButtonID);
    TrackTargetButton.whileHeld(new TrackTarget(camera));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if(AutoPart1command == null){
     AutoPart1command = new RamseteCommand(
      Robot.getAutoPart1Trajectory(), 
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
    }

    if(AutoPart2command == null){
       AutoPart2command = new RamseteCommand(
        Robot.getAutoPart2Trajectory(), 
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

    }
    if (auto == null) {
      auto = new SequentialCommandGroup((AutoPart1command.raceWith(autoIntake))
    .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
    .andThen(stopAndShoot)
    .andThen(AutoPart2command.raceWith(autoIntake))
    .andThen(() -> driveTrain.Drive(0, 0), driveTrain)
    .andThen(backUp.raceWith(autoIntake))
    .andThen(stopAndShoot));
    }
    

    driveTrain.resetOdometry(Robot.getAutoPart1Trajectory().getInitialPose());

    return auto;
  }
}
