package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends ProfiledPIDCommand {

  private Swerve swerve;
  private double heading;
  private static double headingDelta;

  public AutoBalance(Swerve swerve) {
    super(
      new ProfiledPIDController(
        Constants.kBalanceP,
        Constants.kBalanceI,
        Constants.kBalanceD,
        new TrapezoidProfile.Constraints(999, 999)
      ),
      swerve.gyro::getRoll,
      Constants.kLevel,
      (output, setpoint) -> {
        swerve.drive(
          new Translation2d(
            Math.abs(swerve.gyro.getRoll()) < 11 ? 0 : output,
            0
          ),
          Constants.kAngleCorrectionP * headingDelta,
          true,
          false
        );
        System.out.print("Output:" + output);
        System.out.println("Angle" + swerve.gyro.getRoll());
      },
      swerve
    );
    this.swerve = swerve;
    addRequirements(this.swerve);

    getController()
      .setTolerance(
        Constants.kBalanceToleranceDeg,
        Constants.kTurnRateToleranceDegPerS
      );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public void initialize() {
    super.initialize();
    headingDelta = 0;
    heading = swerve.gyro.getAngle();

    if (Math.abs(swerve.gyro.getRoll()) < Constants.kBalanceToleranceDeg) {
      System.out.println(
        "Starting assumption not met: Roll is " +
        swerve.gyro.getPitch() +
        " Tolerance is " +
        Constants.kBalanceToleranceDeg
      );
    }
  }

  @Override
  public void execute() {
    getController()
      .setPID(Constants.kBalanceP, Constants.kBalanceI, Constants.kBalanceD);
    super.execute();
    headingDelta = heading - swerve.gyro.getAngle();
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
