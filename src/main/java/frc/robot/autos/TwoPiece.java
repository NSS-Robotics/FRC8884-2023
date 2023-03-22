package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants;
import frc.robot.commands.nodescoring.*;
import frc.robot.commands.nodescoring.armscoring.*;
import frc.robot.subsystems.*;

public class TwoPiece extends OnePiece {
  public TwoPiece(
    Swerve swerve,
    ClawPivot pivot,
    Claw claw,
    Elevator elevator,
    Arm arm,
    boolean isFirstPath
  ) {
    super(swerve, pivot, claw, elevator, arm, isFirstPath);
  }

  @Override
  public Command followPath() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(
      "TwoPiece",
      new PathConstraints(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
      )
    );

    return new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(claw::openClaw),
        new InstantCommand(pivot::up)
      ),
      new ParallelDeadlineGroup(new WaitCommand(2), new MidNode(elevator)),
      new ParallelDeadlineGroup(new WaitCommand(2), new MidExtend(arm)),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new InstantCommand(claw::closeClaw)
      ),
      new ParallelDeadlineGroup(new WaitCommand(1), new BottomExtend(arm)),
      new ParallelDeadlineGroup(new WaitCommand(2), new BottomNode(elevator)),
      new InstantCommand(() -> {
        // Reset odometry for the first path ran during auto
        if (isFirstPath) {
          swerve.resetOdometry(trajectory.getInitialPose());
        }
      }),
      new ParallelDeadlineGroup(
        new PPSwerveControllerCommand(
          trajectory,
          swerve::getPose,
          Constants.Swerve.swerveKinematics,
          
          new PIDController(
            Constants.Swerve.driveKP,
            Constants.Swerve.driveKI,
            Constants.Swerve.driveKD
          ),
          new PIDController(
            Constants.Swerve.driveKP,
            Constants.Swerve.driveKI,
            Constants.Swerve.driveKD
          ),
          // rotation PID
          new PIDController(
            Constants.Swerve.angleKP,
            Constants.Swerve.angleKI,
            Constants.Swerve.angleKD
          ),
          swerve::setModuleStates,
          // Alter path based on team colour (side of the field)
          true,
          swerve
        )
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(() ->
          swerve.drive(new Translation2d(0, 0), 180, true, false
        ))
      )
    );
  }
}