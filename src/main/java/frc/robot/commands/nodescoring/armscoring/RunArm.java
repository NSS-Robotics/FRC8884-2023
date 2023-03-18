package frc.robot.commands.nodescoring.armscoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class RunArm extends CommandBase {

  private static Arm arm;

  public RunArm(Arm _arm) {
    arm = _arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.runArm();
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("RunArm Command Ended");
  }
}
