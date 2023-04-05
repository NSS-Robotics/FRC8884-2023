package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  private final DoubleSolenoid clawsolenoid;

  private final Compressor compressor;

  protected boolean enabled;
  protected boolean pressureSwitch;
  protected double current;

  public Claw() {
    clawsolenoid = new DoubleSolenoid(14, PneumaticsModuleType.CTREPCM, 4, 5);
    compressor = new Compressor(14, PneumaticsModuleType.CTREPCM);
    enabled = compressor.isEnabled();
    pressureSwitch = compressor.getPressureSwitchValue();
    current = compressor.getCurrent();
  }

  public void openClaw() {
    System.out.println("Claw opened");
    clawsolenoid.set(kReverse);
  }

  public void closeClaw() {
    System.out.println("Claw closed");
    clawsolenoid.set(kForward);
  }

  public void toggle() {
    clawsolenoid.toggle();
  }

  public void startCompressor() {
    compressor.enableDigital();
  }

  public void stopCompressor() {
    compressor.disable();
  }
}
