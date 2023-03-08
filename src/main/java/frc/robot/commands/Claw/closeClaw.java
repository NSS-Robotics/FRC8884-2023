package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class closeClaw extends CommandBase {

  Claw claw;

  public closeClaw(Claw claw) {
    claw = this.claw;
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
