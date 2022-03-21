// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetOdometry extends InstantCommand {
  private DriveTrain driveTrain;
  private Pose2d pose2d;
  /** Creates a new ResetOdometry. */
  public ResetOdometry(DriveTrain driveTrain, Pose2d pose2d) {
    this.driveTrain = driveTrain;
    this.pose2d = pose2d;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetOdometry(pose2d);
  }
}
