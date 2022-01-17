// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.SwingIn;
import frc.robot.commands.SwingOut;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;

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

  private DriveWithJoysticks driveWithJoysticks; 
  private ClimbUp climbUp;
  private ClimbDown climbDown;
  private SwingIn swingIn;
  private SwingOut swingOut;

  private Joystick driverJoystick;
  private Joystick manitulatorJoystick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverJoystick = new Joystick(Constants.driverjoystickID);
    manitulatorJoystick = new Joystick(Constants.manipulatorJoystickID);


    driveTrain = new DriveTrain();

    driveWithJoysticks = new DriveWithJoysticks(driveTrain, driverJoystick);
    driveTrain.setDefaultCommand(driveWithJoysticks);
    climbUp = new ClimbUp(climber);
    climbDown = new ClimbDown(climber);


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
    JoystickButton climbUpButton = new JoystickButton(manitulatorJoystick, Constants.climbUpButtonID);
    climbUpButton.whileHeld(climbUp);

    JoystickButton climbDownButton = new JoystickButton(manitulatorJoystick, Constants.climbDownButtonID);
    climbDownButton.whileHeld(climbDown);

    JoystickButton swingInButton = new JoystickButton(manitulatorJoystick, Constants.swingInButtonID);
    swingInButton.whileHeld(swingIn);

    JoystickButton swingOutButton = new JoystickButton(manitulatorJoystick, Constants.swingOutButtonID);
    swingOutButton.whileHeld(swingOut);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
