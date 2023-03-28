package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AlignGyro;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AlignLimelight extends SequentialCommandGroup {

  private final Limelight limelight;
  private final Swerve swerve;

  public AlignLimelight(Limelight limelight, Swerve swerve) {
    this.limelight = limelight;
    this.swerve = swerve;
    addRequirements(this.limelight, this.swerve);

    addCommands(
      new AlignGyro(swerve),
      new LateralAlignLimelight(swerve, limelight)
      //new GoToLimelight(swerve, limelight)
    );
  }
}







