package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AlignGyro;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Target;
import frc.robot.subsystems.Swerve;

public class AlignLimelight extends SequentialCommandGroup {

  private Target target;
  private final Limelight limelight;
  private final Swerve swerve;

  public AlignLimelight(Target target, Limelight limelight, Swerve swerve) {
    this.limelight = limelight;
    this.swerve = swerve;
    this.target = target;
    addRequirements(this.limelight, this.swerve);

    double setpoint = this.target == Target.HP ? 0.0 : 180.0;

    addCommands(
      new InstantCommand(() -> limelight.configure(this.target)),
      new AlignGyro(setpoint, swerve),
      new LateralAlignLimelight(swerve, limelight)
    );
  }
}
