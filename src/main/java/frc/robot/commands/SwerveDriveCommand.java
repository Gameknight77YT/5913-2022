package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private final Joystick driverJoystick;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);

  public SwerveDriveCommand(SwerveDrivetrain dt, Joystick joy) {
    drivetrain = dt;
    addRequirements(drivetrain);

    driverJoystick = joy;
  }

  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
      xspeedLimiter.calculate(driverJoystick.getX())
        * Units.feetToMeters(Constants.MaxSpeedMetersPerSecond);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
      yspeedLimiter.calculate(driverJoystick.getY())
        * Units.feetToMeters(Constants.MaxSpeedMetersPerSecond);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
      rotLimiter.calculate(driverJoystick.getZ())
        * Units.feetToMeters(Constants.MaxAccelerationMetersPerSecondSquared);

    

    drivetrain.drive(xSpeed, ySpeed, rot, true);
    
  }

}