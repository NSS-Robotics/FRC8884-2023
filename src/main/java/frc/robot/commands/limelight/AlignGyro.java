package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class AlignGyro extends PIDCommand {

  public AlignGyro(
    PIDController controller,
    DoubleSupplier measurementSource,
    double setpoint,
    DoubleConsumer useOutput,
    Subsystem[] requirements
  ) {
    super(controller, measurementSource, setpoint, useOutput, requirements);
  }

  public AlignGyro(Swerve swerve) {
    super(
      new PIDController(0.05, 0.0, 0.01),
      swerve.gyro::getYaw,
      180.0,
      angle -> swerve.turnStates(angle),
      swerve
    );
    addRequirements(swerve);
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Constants.turnTolerance);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }
}
