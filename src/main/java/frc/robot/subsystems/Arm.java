package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private static CANSparkMax Motor;
  private static RelativeEncoder MotorEncoder;
  private static SparkMaxPIDController Motorpid;

  public boolean armReset = false;

  public void armSetup() {
    // Motor Setup
    Motor = new CANSparkMax(Constants.ArmConstants.MotorID, MotorType.kBrushless);
    Motor.restoreFactoryDefaults();
    Motor.setIdleMode(IdleMode.kBrake);
    Motor.setSmartCurrentLimit(40);
    Motor.setOpenLoopRampRate(0.5);
    Motor.setClosedLoopRampRate(0.5);
    MotorEncoder = Motor.getEncoder();
    Motorpid = Motor.getPIDController();
    Motorpid.setP(Constants.ArmConstants.Kp);
    Motorpid.setI(Constants.ArmConstants.Ki);
    Motorpid.setD(Constants.ArmConstants.Kd);
    Motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    Motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    Motor.setSoftLimit(SoftLimitDirection.kForward, Constants.ArmConstants.ExtendMax);
    Motor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    resetArmEncoders();
  }

  public void resetArmEncoders() {
    MotorEncoder.setPosition(0);
  }

  public void setArm(double value) {
    Motorpid.setReference(value, ControlType.kPosition, 0);
  }

  public void disableArmLimits() {
    Motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    Motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public void enableArmLimits() {
    Motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    Motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public void resetArm() {
    armReset = true;
    MotorEncoder.setPosition(0);
  }

  public void runArm() {
    Motor.set(0.2);
  }

  public void stopArm() {
    Motor.set(0);
  }

  public double[] getArmCurrent() {
    double outputcurrent[] = new double[2];
    outputcurrent[0] = Motor.getOutputCurrent();
    return outputcurrent;
  }

  public double getArmEncoder() {
    return MotorEncoder.getPosition();
  }

  public Arm() {
    armSetup();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ArmMotorEncoder", MotorEncoder.getPosition());
  }
}
