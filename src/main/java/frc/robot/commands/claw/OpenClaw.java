package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class OpenClaw extends CommandBase {

  private final Claw claw;

  public OpenClaw(Claw _claw) {
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
