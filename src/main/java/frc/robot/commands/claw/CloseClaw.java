package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawPivot;

public class CloseClaw extends CommandBase {

  private final Claw claw;

  public CloseClaw(Claw _claw) {
    claw = _claw;
    addRequirements(claw);
  }

  @Override
  public void execute() {
    claw.closeClaw();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Claw Close");
  }
}
