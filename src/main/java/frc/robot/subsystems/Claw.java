package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  private DoubleSolenoid clawsolenoid;
  private Compressor compressor;

  public Claw() {
    clawsolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    compressor = new Compressor(14, PneumaticsModuleType.CTREPCM);
    boolean enabled = compressor.isEnabled();
    boolean pressureSwitch = compressor.getPressureSwitchValue();
    double current = compressor.getCurrent();
  }

  public void openClaw() {
    clawsolenoid.set(Value.kReverse);
  }

  public void closeClaw() {
    clawsolenoid.set(Value.kForward);
  }

  public void stopClaw() {
    clawsolenoid.set(Value.kOff);
  }

  public void startCompressor() {
    compressor.enableDigital();
  }

  public void stopCompressor() {
    compressor.disable();
  }
}
