package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class openClaw extends CommandBase {

  private final Claw claw;

  public openClaw(Claw _claw) {
    claw = _claw;
    addRequirements(claw);
  }

  @Override
  public void execute() {
    claw.openClaw();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Claw Open");
  }
}
