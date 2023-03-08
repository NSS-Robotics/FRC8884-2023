package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargePadCurved extends CommandBase {
  private Swerve swerve;

  public ChargePadCurved(Swerve swerve) {
    swerve = this.swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  // loads path and executes it, here you can alter speed etc.
  @Override
  public void execute() {
    swerve.FollowPath(
        PathPlanner.loadPath(
            "ChargePathCurved",
            new PathConstraints(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)),
        true);
  }

  @Override
  public void end(boolean interrupted) {}

  // true when command ends successfully
  @Override
  public boolean isFinished() {
    return false;
  }
}
