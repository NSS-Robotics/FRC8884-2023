package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawPivot extends SubsystemBase {

  private final DoubleSolenoid pivotsolenoid;

  public ClawPivot() {
    pivotsolenoid = new DoubleSolenoid(14, PneumaticsModuleType.CTREPCM, 3, 2);
  }

  public void up() {
    pivotsolenoid.set(kForward);
  }

  public void down() {
    pivotsolenoid.set(kReverse);
  }
}
