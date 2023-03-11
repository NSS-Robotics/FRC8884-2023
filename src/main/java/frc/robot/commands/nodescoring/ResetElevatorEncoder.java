package frc.robot.commands.nodescoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ResetElevatorEncoder extends CommandBase {

  private final Elevator elevator;
  private int currentCounter = 0;

  public ResetElevatorEncoder(Elevator _elevator) {
    elevator = _elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.disableElevatorLimits();
    currentCounter = 0;
  }

  @Override
  public void execute() {
    elevator.setElevatorSpeed(1.65); // 1.65 is magic number
    for (double i : elevator.getElevatorCurrent()) {
      if (i >= 20.0) {
        currentCounter++;
        break;
      }
    }
  }

  @Override
  public boolean isFinished() {
    // return ShooterSubsystem.getHoodCurrent() >= Constants.NEO550_CURRENT_SPIKE &&
    // Math.abs(ShooterSubsystem.getHoodVelocity()) <= 10;
    return currentCounter >= 3;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.resetElevator();
    elevator.stopElevator();
    elevator.enableElevatorLimits();
  }
}
