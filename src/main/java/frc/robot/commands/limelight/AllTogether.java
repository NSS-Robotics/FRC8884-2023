package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.AlignGyro;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Target;
import frc.robot.subsystems.Swerve;

public class AllTogether extends SequentialCommandGroup {

  private Target target;
  private final Limelight limelight;
  private final Swerve swerve;

  public AllTogether(Target target, Limelight limelight, Swerve swerve) {
    this.limelight = limelight;
    this.swerve = swerve;
    this.target = target;
    addRequirements(this.limelight, this.swerve);

    double setpoint = this.target == Target.HP ? 0.0 : 180.0;

    addCommands(
      new InstantCommand(() -> {
        if (this.target == Target.Cone) {
          limelight.setPipeline(1);
          limelight.setTargetHeight(Constants.tapeHeight);
        } else if (this.target == Target.Cube) {
          limelight.setPipeline(0);
          limelight.setTargetHeight(Constants.tagHeight);
        } else {
          limelight.setPipeline(0);
          limelight.setTargetHeight(Constants.playerStationHeight);
        }
      }),
      new AimLimelight(swerve, limelight),
      new GoToLimelight(swerve, limelight),
      new AlignGyro(setpoint, swerve)
    );
  }
}
