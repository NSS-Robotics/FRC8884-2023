package frc.robot.commands.nodescoring.armscoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class TopExtend extends CommandBase {

  private static Arm arm;

  public TopExtend(Arm _arm) {
    arm = _arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.setArm(Constants.ArmConstants.ExtendTopNode);
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("TopExtend Command Ended");
  }
}
