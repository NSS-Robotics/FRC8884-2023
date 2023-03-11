package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class closeClaw extends CommandBase {

  private final Claw claw;

  public closeClaw(Claw _claw) {
    claw = _claw;
    addRequirements(claw);
  }

  @Override
  public void execute() {
    claw.closeClaw();
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("BottomNode Command Ended");
  }
}
