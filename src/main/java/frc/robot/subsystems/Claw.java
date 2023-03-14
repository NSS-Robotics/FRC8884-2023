package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  private final DoubleSolenoid clawsolenoid;

  // private final Compressor compressor;

  public Claw() {
    clawsolenoid = new DoubleSolenoid(14, PneumaticsModuleType.CTREPCM, 0, 1);
    // compressor = new Compressor(14, PneumaticsModuleType.CTREPCM);
    // boolean enabled = compressor.isEnabled();
    // boolean pressureSwitch = compressor.getPressureSwitchValue();
    // double current = compressor.getCurrent();
  }

  public void openClaw() {
    clawsolenoid.set(kReverse);
  }

  public void closeClaw() {
    clawsolenoid.set(kForward);
  }
  // public void startCompressor() {
  //   compressor.enableDigital();
  // }

  // public void stopCompressor() {
  //   compressor.disable();
  // }
}
