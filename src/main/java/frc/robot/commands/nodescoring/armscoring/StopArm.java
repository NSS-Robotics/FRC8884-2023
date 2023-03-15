package frc.robot.commands.nodescoring.armscoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class StopArm extends CommandBase {

  private static Arm arm;

  public StopArm(Arm _arm) {
    arm = _arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.stopArm();
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("Stop Arm Command Ended");
  }
}
