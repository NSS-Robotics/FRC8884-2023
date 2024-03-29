package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.claw.*;
import frc.robot.commands.nodescoring.*;
import frc.robot.commands.nodescoring.armscoring.*;
import frc.robot.subsystems.*;

public class TwoPiecePlsWork extends CommandBase {

  protected boolean isFirstPath;

  protected final Swerve swerve;
  protected final ClawPivot pivot;
  protected final Claw claw;
  protected final Elevator elevator;
  protected final Arm arm;

  public TwoPiecePlsWork(
      Swerve swerve, ClawPivot pivot, Claw claw, Elevator elevator, Arm arm, boolean isFirstPath) {
    this.swerve = swerve;
    this.pivot = pivot;
    this.claw = claw;
    this.elevator = elevator;
    this.arm = arm;
    this.isFirstPath = isFirstPath;
    addRequirements(swerve, pivot, claw, elevator, arm);
  }

  @Override
  public void initialize() {}

  public Command followPath() {
    PathPlannerTrajectory trajectory =
        PathPlanner.loadPath(
            "TwoPiece",
            new PathConstraints(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new WaitCommand(1), new InstantCommand(claw::openClaw), new InstantCommand(pivot::up)),
        new ParallelDeadlineGroup(new WaitCommand(2), new MidNode(elevator)),
        new ParallelDeadlineGroup(new WaitCommand(2), new MidExtend(arm)),
        new ParallelDeadlineGroup(new WaitCommand(0.5), new InstantCommand(claw::closeClaw)),
        new ParallelDeadlineGroup(new WaitCommand(1), new BottomExtend(arm)),
        new ParallelDeadlineGroup(new WaitCommand(2), new BottomNode(elevator)),
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
            swerve));
  }
}
