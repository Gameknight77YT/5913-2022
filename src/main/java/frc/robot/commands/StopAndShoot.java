// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Shooter;

public class StopAndShoot extends CommandBase {
  Camera camera;
  Shooter shooter;
  Timer timer = new Timer();
  boolean finished = false;
  /** Creates a new StopAndShoot. */
  public StopAndShoot(Shooter s, Camera c) {
    shooter = s;
    camera = c;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    while(timer.get() <= Constants.stopAndShootTime){
      camera.Track();
      shooter.controlIntake(Constants.intakeSpeed, Constants.intakeSystemSpeed, Constants.feederSpeed);
      shooter.shootBall(1, 0);
    }
    finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    shooter.stopIntake();
    camera.Reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
