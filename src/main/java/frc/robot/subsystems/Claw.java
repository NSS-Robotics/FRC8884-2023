package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private CANSparkMax clawMotor;
    private DoubleSolenoid clawSolenoid;
    private Compressor compressor;

    public Claw() {
        clawMotor = new CANSparkMax(0, MotorType.kBrushless);
        clawMotor.setIdleMode(IdleMode.kBrake);
        clawMotor.setSmartCurrentLimit(40);
        clawMotor.setOpenLoopRampRate(0.5);
        clawMotor.setClosedLoopRampRate(0.5);

        clawSolenoid = new DoubleSolenoid(null, 0, 1);
        compressor = new Compressor(0, null);
    }

    public void openClaw() {
        clawSolenoid.set(Value.kForward);
    }

    public void closeClaw() {
        clawSolenoid.set(Value.kReverse);
    }

    public void stopClaw() {
        clawSolenoid.set(Value.kOff);
    }

    public void runClaw(double speed) {
        clawMotor.set(speed);
    }

    public void stop() {
        clawMotor.set(0);
    }

    public void startCompressor() {
        compressor.enableAnalog(0, 0);
    }

    public void stopCompressor() {
        compressor.disable();
    }
}