package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Elevator extends SubsystemBase {
  public CANSparkMax Lmotor;
  public CANSparkMax Rmotor;

  public Elevator() {
        Lmotor = new CANSparkMax(12, MotorType.kBrushless);
        Lmotor.setIdleMode(IdleMode.kBrake);
        Lmotor.setSmartCurrentLimit(40);
        Lmotor.setOpenLoopRampRate(0.5);
        Lmotor.setClosedLoopRampRate(0.5);

        Rmotor = new CANSparkMax(13, MotorType.kBrushless);
        Rmotor.setIdleMode(IdleMode.kBrake);
        Rmotor.setSmartCurrentLimit(40);
        Rmotor.setOpenLoopRampRate(0.5);
        Rmotor.setClosedLoopRampRate(0.5);
  }

  public void Extend() {
    Lmotor.set(.3);
    Rmotor.set(.3);

  }
  
  public void Retract() {
    Lmotor.set(-.3);
    Rmotor.set(-.3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}