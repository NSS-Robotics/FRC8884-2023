package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.commands.claw.*;
import frc.robot.commands.nodescoring.*;
import frc.robot.commands.nodescoring.armscoring.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;


public class TwoPiecePlsWork extends OnePiecePlsWork {
  public TwoPiecePlsWork(
    Swerve swerve, ClawPivot pivot, Claw claw,
    Elevator elevator, Arm arm, boolean isFirstPath
  ) {
    super(swerve, pivot, claw, elevator, arm, isFirstPath);
  }

  @Override
  public Command followPath() {
    PathPlannerTrajectory trajectory =
        PathPlanner.loadPath(
            "TwoPiecePlsWork",
            new PathConstraints(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new PivotDown(pivot),
            new TopNode(elevator),
            new TopExtend(arm),
            new OpenClaw(claw)
        ),
        new ParallelDeadlineGroup(
            new WaitCommand(2),
            new BottomExtend(arm),
            new BottomNode(elevator)
        ),
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
            swerve),
        new InstantCommand(
          () -> swerve.drive(new Translation2d(0, 0), 180, false, true)
        ),
        new OpenClaw(claw));
  }
}