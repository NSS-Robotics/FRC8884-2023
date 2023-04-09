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
import frc.robot.commands.nodescoring.*;
import frc.robot.commands.nodescoring.armscoring.*;
import frc.robot.subsystems.*;

public class StandStill extends CommandBase {

  public boolean isFirstPath = true;
  public boolean isMidNode = false;

  protected final Swerve swerve;
  protected final ClawPivot pivot;
  protected final Claw claw;
  protected final Elevator elevator;
  protected final Arm arm;

  public StandStill(
    Swerve swerve,
    ClawPivot pivot,
    Claw claw,
    Elevator elevator,
    Arm arm,
    boolean isFirstPath
  ) {
    this.swerve = swerve;
    this.pivot = pivot;
    this.claw = claw;
    this.elevator = elevator;
    this.arm = arm;
    this.isFirstPath = isFirstPath;
    addRequirements(swerve, pivot, claw, elevator, arm);
  }

  public StandStill isMidNode(boolean t) {
    this.isMidNode = t;
    return this;
  }

  @Override
  public void initialize() {}

  public Command followPath() {
    Command elevatorNode = new TopNode(elevator);
    Command armNode = new TopExtend(arm);

    return new SequentialCommandGroup(
      new InstantCommand(swerve::zeroGyro),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(claw::closeClaw),
        new InstantCommand(pivot::down)
      ),
      new ParallelDeadlineGroup(new WaitCommand(2), elevatorNode),
      new ParallelDeadlineGroup(new WaitCommand(2.5), armNode),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new InstantCommand(claw::openClaw)
      ),
      new ParallelDeadlineGroup(new WaitCommand(2), new FullyRetract(arm)),
      new ParallelDeadlineGroup(new WaitCommand(1.5), new BottomNode(elevator)),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new InstantCommand(pivot::up)
      )
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
