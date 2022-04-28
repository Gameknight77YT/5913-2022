// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  private DriveTrain driveTrain;
  private double target, angle, error;
  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain driveTrain, double target) {
    this.driveTrain = driveTrain; 
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = Robot.getHeading().getDegrees() + target;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = Robot.getHeading().getDegrees();
    error = target - angle;
    if(error > 1){
      driveTrain.Drive(-.25, .25);//right turn
    }else if(error < 1){
      driveTrain.Drive(.25, -.25);//left turn
    }else{
      driveTrain.Drive(0, 0);
    }
    SmartDashboard.putNumber("error", error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.Drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(error) < 2.5){
      return true;
    }else{
      return false;
    }
    
  }
}
