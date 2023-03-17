package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.claw.*;
import frc.robot.commands.nodescoring.*;
import frc.robot.commands.nodescoring.armscoring.*;
import frc.robot.subsystems.*;
import java.util.HashMap;
import java.util.List;

public class OnePiece extends CommandBase {

  protected boolean isFirstPath;
  protected HashMap<String, Command> eventMap;

  private static Swerve _swerve;
  protected final ClawPivot pivot;
  protected final Claw claw;
  protected final Elevator elevator;
  protected final Arm arm;

  public OnePiece(
    Swerve _swerve,
    ClawPivot pivot,
    Claw claw,
    Elevator elevator,
    Arm arm,
    boolean isFirstPath,
    HashMap<String, Command> eventMap
  ) {
    Swerve swerve = _swerve;
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
    List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup(
      "Test",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
    );
    eventMap.clear();
    eventMap.put("Up", new MidNode(elevator).asProxy());
    eventMap.put("Out", new MidExtend(arm).asProxy());
    return RobotContainer.BuildAuto(trajectory);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
