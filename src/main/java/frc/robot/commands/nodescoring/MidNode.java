package frc.robot.commands.nodescoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class MidNode extends CommandBase{
    Elevator elevator;

    public MidNode(Elevator _elevator) {
        elevator = _elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setElevator(Constants.ElevatorConstants.MidNodeDistance);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("MidNode Command Ended");    
    }
}
