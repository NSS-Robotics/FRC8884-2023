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
    private CANSparkMax LMotor;
    private CANSparkMax RMotor;
    private DoubleSolenoid LSolenoid;
    private DoubleSolenoid RSolenoid;
    private DoubleSolenoid clawSolenoid;
    private Compressor compressor;

    public Claw() {
        LMotor = new CANSparkMax(0, MotorType.kBrushless);
        LMotor.setIdleMode(IdleMode.kBrake);
        LMotor.setSmartCurrentLimit(40);
        LMotor.setOpenLoopRampRate(0.5);
        LMotor.setClosedLoopRampRate(0.5);

        RMotor = new CANSparkMax(0, MotorType.kBrushless);
        RMotor.setIdleMode(IdleMode.kBrake);
        RMotor.setSmartCurrentLimit(40);
        RMotor.setOpenLoopRampRate(0.5);
        RMotor.setClosedLoopRampRate(0.5);

        clawSolenoid = new DoubleSolenoid(null, 0, 1);
        compressor = new Compressor(0, null);
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

    public void runClaw(double speed) {
        LMotor.set(speed);
        RMotor.set(speed);
    }

    public void stop() {
        LMotor.set(0);
        RMotor.set(0);
    }

    public void startCompressor() {
        compressor.enableAnalog(0, 0);
    }

    public void stopCompressor() {
        compressor.disable();
    }
}