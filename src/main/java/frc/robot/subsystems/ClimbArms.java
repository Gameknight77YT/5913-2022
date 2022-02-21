// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbArms extends SubsystemBase {
  DoubleSolenoid swingSolenoid = new DoubleSolenoid(Constants.pcmID, PneumaticsModuleType.CTREPCM, Constants.swingForwardID, Constants.swingReverseID);
  /** Creates a new ClimbArms. */
  public ClimbArms() {
    //swingSolenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void swingOut() {
    swingSolenoid.set(Value.kReverse);
  }

  public void swingIn() {
    swingSolenoid.set(Value.kForward);
  }
  
}
