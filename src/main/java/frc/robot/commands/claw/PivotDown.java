package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawPivot;

public class PivotDown extends CommandBase {

  private final ClawPivot pivot;

  public PivotDown(ClawPivot _pivot) {
    pivot = _pivot;
    addRequirements(pivot);
  }

  @Override
  public void execute() {
    pivot.down();
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("Pivot Down");
  }
}
