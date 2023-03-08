package frc.robot.commands.ArmLengths;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class TopExtend extends CommandBase {

  Arm arm;

  public TopExtend(Arm arm) {
    arm = this.arm;
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
