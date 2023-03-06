package frc.robot.commands.nodescoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class MidNode extends CommandBase {

  private final Elevator elevator;

  public MidNode(Elevator _elevator) {
    elevator = _elevator;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.setElevator(Constants.ElevatorConstants.MidNodeDistance);
    /*
    if (
      elevator.getElevatorEncoder()[0] <
      Constants.ElevatorConstants.MidNodeDistance &&
      elevator.getElevatorEncoder()[1] <
      Constants.ElevatorConstants.MidNodeDistance
    ) {
      elevator.setElevatorSpeed(0.1);
    }
    */
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("MidNode Command Ended");
  }
}
