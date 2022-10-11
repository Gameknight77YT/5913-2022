// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StopAndShoot extends CommandBase {
  Camera camera;
  Shooter shooter;
  Intake intake;
  Timer timer = new Timer();
  boolean finished = false;
  double waitTime, shootTime;
  /** Creates a new StopAndShoot. */
  public StopAndShoot(Shooter s, Camera c, Intake i, double waitTime, double shootTime) {
    shooter = s;
    camera = c;
    intake = i;
    this.waitTime = waitTime;
    this.shootTime = shootTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, camera, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    while(timer.get() <= waitTime){
      camera.Track();
      intake.controlIntake(Constants.intakeSpeed, Constants.starfishSpeed, 0);
      shooter.shootBall(5, 0);
    }
    while(timer.get() <= waitTime + shootTime){
      camera.Track();
      intake.controlIntake(Constants.intakeSpeed, Constants.starfishSpeed, Constants.feederSpeed);
      shooter.shootBall(5, 0);
    }
    camera.Control(0);
    finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    intake.stopIntake();
    camera.Reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
