package frc.robot.commands.nodescoring.armscoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MidExtend extends CommandBase {

  Arm arm;

  public MidExtend(Arm arm) {
    arm = this.arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {}

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("MidExtend Command Ended");
  }
}
