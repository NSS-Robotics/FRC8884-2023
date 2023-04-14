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

/** horizontally aligns the robot using the TX value */
public class LateralAlignLimelight extends PIDCommand {

  private Swerve swerve;

  public LateralAlignLimelight(
    PIDController controller,
    DoubleSupplier measurementSource,
    double setpoint,
    DoubleConsumer useOutput,
    Subsystem[] requirements
  ) {
    super(controller, measurementSource, setpoint, useOutput, requirements);
  }

  public LateralAlignLimelight(Swerve _swerve, Limelight limelight) {
    super(
      new PIDController(
        Constants.kLateralP,
        Constants.kLateralI,
        Constants.kLateralD
      ),
      limelight::lateralDistance,
      Constants.mountOffset,
      distance -> {
        _swerve.drive(new Translation2d(0, distance / -100), 0, true, false);
      },
      _swerve
    );
    swerve = _swerve;
    addRequirements(swerve, limelight);
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Constants.turnTolerance);

    System.out.println("Align With Limelight - Start");
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, false, false);
    System.out.println("Align With Limelight - End");

    super.end(interrupted);
  }
}
