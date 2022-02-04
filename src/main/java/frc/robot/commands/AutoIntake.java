// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends CommandBase {
  Camera camera;
  Shooter shooter;
  /** Creates a new AutoIntake. */
  public AutoIntake(Camera c, Shooter s) {
    shooter = s;
    camera = c;
    addRequirements(shooter, camera);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.toggleIntakeArms(1);
    shooter.controlIntake(Constants.intakeSpeed,Constants.intakeSystemSpeed, 0);
    shooter.shootBall(1, 0);
    camera.AutoTrack();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopIntake();
    camera.Reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
