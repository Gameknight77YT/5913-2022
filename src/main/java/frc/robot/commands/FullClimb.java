// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class FullClimb extends CommandBase {
  private Climber climber;
  private boolean finished = false;
  /** Creates a new FullClimb. */
  public FullClimb(Climber c) {
    climber = c;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    while(climber.getClimbEncoder() > 0){
      climber.setClimberMotor(-Constants.climberSpeed);
    }
    climber.setClimberMotor(0);
    climber.swingOut();
    while(climber.getClimbEncoder() < Constants.climbEncoderTop){
      climber.setClimberMotor(Constants.climberSpeed);
    }
    climber.setClimberMotor(0);
    climber.swingIn();
    while(climber.getClimbEncoder() > 0){
      climber.setClimberMotor(-Constants.climberSpeed);
    }
    climber.setClimberMotor(0);
    climber.swingOut();
    while(climber.getClimbEncoder() < Constants.climbEncoderTop){
      climber.setClimberMotor(Constants.climberSpeed);
    }
    climber.setClimberMotor(0);
    climber.swingIn();
    while(climber.getClimbEncoder() < Constants.climbEncoderTop/2){
      climber.setClimberMotor(Constants.climberSpeed);
    }
    climber.setClimberMotor(0);
    finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
