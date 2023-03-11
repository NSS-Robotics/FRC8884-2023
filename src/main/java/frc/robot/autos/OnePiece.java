package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OnePiece extends CommandBase {
  private Swerve swerve;
  private boolean isFirstPath;

  public OnePiece(Swerve swerve, boolean isFirstPath) {
    this.swerve = swerve;
    this.isFirstPath = isFirstPath;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  public Command followPath() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(
      "OnePiece",
      new PathConstraints(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
      )
    );

    return new SequentialCommandGroup(
      new InstantCommand(
        () -> {
          // Reset odometry for the first path ran during auto
          if (isFirstPath) {
            swerve.resetOdometry(trajectory.getInitialPose());
          }
        }),
      new PPSwerveControllerCommand(
        trajectory,
        swerve::getPose,
        Constants.Swerve.swerveKinematics,
        // XY PID drive values, usually same
        new PIDController(
            Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD),
        new PIDController(
            Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD),
        // rotation PID
        new PIDController(
            Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD),
        swerve::setModuleStates,
        // Alter path based on team colour (side of the field)
        true,
        swerve
      )
    );
  }

  @Override
  public void end(boolean interrupted) {}

  // true when command ends successfully
  @Override
  public boolean isFinished() {
    return false;
  }
}
