package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.claw.*;
import frc.robot.commands.nodescoring.*;
import frc.robot.commands.nodescoring.armscoring.*;
import frc.robot.subsystems.*;
import java.util.HashMap;

public class TwoPiece extends CommandBase {

  protected boolean isFirstPath;
  protected HashMap<String, Command> eventMap;

  protected final Swerve swerve;
  protected final ClawPivot pivot;
  protected final Claw claw;
  protected final Elevator elevator;
  protected final Arm arm;

  public TwoPiece(
      Swerve swerve,
      ClawPivot pivot,
      Claw claw,
      Elevator elevator,
      Arm arm,
      boolean isFirstPath,
      HashMap<String, Command> eventMap) {
    this.swerve = swerve;
    this.pivot = pivot;
    this.claw = claw;
    this.elevator = elevator;
    this.arm = arm;
    this.isFirstPath = isFirstPath;
    this.eventMap = eventMap;
    addRequirements(swerve, pivot, claw, elevator, arm);
  }

  @Override
  public void initialize() {}

  public Command followPath() {
    PathPlannerTrajectory trajectory =
        PathPlanner.loadPath(
            "TwoPiece",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    eventMap.put(
        "Close Claw", new SequentialCommandGroup(new CloseClaw(claw), new PivotDown(pivot)));
    eventMap.put(
        "Score",
        new SequentialCommandGroup(new TopNode(elevator), new TopExtend(arm), new OpenClaw(claw)));
    eventMap.put(
        "Bottom Node", new SequentialCommandGroup(new BottomNode(elevator), new BottomExtend(arm)));

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
            swerve));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
