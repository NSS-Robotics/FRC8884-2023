package frc.robot.commands.nodescoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class TopNode extends CommandBase {

  private final Elevator elevator;

  public TopNode(Elevator _elevator) {
    elevator = _elevator;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.setElevator(Constants.ElevatorConstants.TopNodeDistance);
    /*
    if (
      elevator.getElevatorEncoder()[0] <
      Constants.ElevatorConstants.TopNodeDistance &&
      elevator.getElevatorEncoder()[1] <
      Constants.ElevatorConstants.TopNodeDistance
    ) {
      elevator.setElevatorSpeed(0.1);
    }
    */
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("TopNode Command Ended");
  }
}
