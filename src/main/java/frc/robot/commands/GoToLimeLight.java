package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class GoToLimeLight extends PIDCommand {

  private Swerve swerve;

  public GoToLimeLight(
    PIDController controller,
    DoubleSupplier measurementSource,
    double setpoint,
    DoubleConsumer useOutput,
    Subsystem[] requirements
  ) {
    super(controller, measurementSource, setpoint, useOutput, requirements);
  }

  public GoToLimeLight(Swerve s_Swerve, Limelight limelight) {
    super(
      new PIDController(Constants.turn_P, Constants.turn_I, Constants.turn_D),
      limelight::estimateDistance,
      10,
      x -> s_Swerve.drive(new Translation2d(x, 0), 0, true, false),
      s_Swerve
    );
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Constants.turnTolerance);

    System.out.println("Align With Limelight - Start");
    swerve = s_Swerve;
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, false, false);
    System.out.println("Align With Limelight - End");
  }
}
/** A command that will turn the robot to the specified angle. */
