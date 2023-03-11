package frc.robot.commands.armscoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MidExtend extends CommandBase {

  Arm arm;

  public MidExtend(Arm arm) {
    arm = this.arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.setArm(Constants.ArmConstants.ExtendMidNode);
    if (arm.getArmEncoder() < Constants.ArmConstants.ExtendMidNode) {
      arm.setArmSpeed(0.1);
    }
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("MidExtend Command Ended");
  }
}
