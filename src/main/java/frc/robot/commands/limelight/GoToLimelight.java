package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/** A command that will turn drive the robot to the target. */
public class GoToLimelight extends PIDCommand {

  private Swerve swerve;

  public GoToLimelight(
    PIDController controller,
    DoubleSupplier measurementSource,
    double setpoint,
    DoubleConsumer useOutput,
    Subsystem[] requirements
  ) {
    super(controller, measurementSource, setpoint, useOutput, requirements);
  }

  public GoToLimelight(Swerve _swerve, Limelight limelight) {
    super(
      new PIDController(Constants.lateral_P, Constants.lateral_I, Constants.lateral_D),
      limelight::estimateDistance,
      Constants.distToScoring,
      x -> _swerve.drive(new Translation2d(x / -100, 0), 0, true, false),
      _swerve
    );
    swerve = _swerve;
    addRequirements(swerve, limelight);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Constants.turnTolerance);

    System.out.println("Limelight go to target - start");
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, false, false);
    System.out.println("Limelight go to target - end");
  }
}
