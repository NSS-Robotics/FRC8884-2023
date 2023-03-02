package frc.robot.commands.ArmLengths;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class BottomExtend extends CommandBase {

    Arm arm;

    public BottomExtend(Arm arm) {
        arm = this.arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setArm(Constants.ArmConstants.ExtendBottomNode);
    }

    @Override
    public void initialize() {}

    @Override
    public void end(boolean interrupted) {
        System.out.println("BottomExtend Command Ended");
    }
}
