package frc.robot.commands.nodescoring.armscoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class FullyRetract extends CommandBase {

  private static Arm arm;

  public FullyRetract(Arm _arm) {
    arm = _arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.setArm(0);
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("FullyRetract Command Ended");
  }
}
