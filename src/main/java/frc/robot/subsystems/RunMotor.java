// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMax.SoftLimitDirection;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class RunMotor extends SubsystemBase {

//   private CANSparkMax Lmotor;
//   private CANSparkMax Rmotor;

//   public RunMotor() {
//     Lmotor = new CANSparkMax(12, MotorType.kBrushless);
//     Lmotor.setIdleMode(IdleMode.kBrake);
//     Lmotor.setSmartCurrentLimit(50);
//     Lmotor.setOpenLoopRampRate(0.3);
//     Lmotor.setClosedLoopRampRate(0.3);

//     Rmotor = new CANSparkMax(13, MotorType.kBrushless);
//     Rmotor.setIdleMode(IdleMode.kBrake);
//     Rmotor.setSmartCurrentLimit(50);
//     Rmotor.setOpenLoopRampRate(0.3);
//     Rmotor.setClosedLoopRampRate(0.3);
//   }

//   public void Extend() {
//     Lmotor.set(-.5);
//     Rmotor.set(.5);
//   }

//   public void Retract() {
//     Lmotor.set(.5);
//     Rmotor.set(-.5);
//   }

//   public void Stop() {
//     Lmotor.set(0);
//     Rmotor.set(0);
//   }
// }
