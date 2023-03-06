package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  private DoubleSolenoid LSolenoid;
  private DoubleSolenoid RSolenoid;
  private Compressor compressor;

  public Claw() {
    LSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    RSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    compressor = new Compressor(14, PneumaticsModuleType.CTREPCM);
    boolean enabled = compressor.isEnabled();
    boolean pressureSwitch = compressor.getPressureSwitchValue();
    double current = compressor.getCurrent();
  }

  public void openClaw() {
    LSolenoid.set(Value.kForward);
    RSolenoid.set(Value.kForward);
  }

  public void closeClaw() {
    LSolenoid.set(Value.kReverse);
    RSolenoid.set(Value.kReverse);
  }

  public void stopClaw() {
    LSolenoid.set(Value.kOff);
    RSolenoid.set(Value.kOff);
  }

  public void startCompressor() {
    compressor.enableDigital();
  }

  public void stopCompressor() {
    compressor.disable();
  }
}
