package frc.robot.commands.nodescoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class BottomNode extends CommandBase {

  Elevator elevator;

  public BottomNode(Elevator _elevator) {
    elevator = _elevator;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.setElevator(Constants.ElevatorConstants.BottomNodeDistance);
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("BottomNode Command Ended");
  }
}
