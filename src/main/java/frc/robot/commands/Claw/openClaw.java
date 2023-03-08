package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class openClaw extends CommandBase {

  Claw claw;

  public openClaw(Claw claw) {
    claw = this.claw;
    addRequirements(claw);
  }

  @Override
  public void execute() {
    claw.openClaw();
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("BottomNode Command Ended");
  }
}
