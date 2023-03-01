package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax Lmotor;
  private CANSparkMax Rmotor;
  private RelativeEncoder LmotorEncoder;
  private RelativeEncoder RmotorEncoder;
  private SparkMaxPIDController Lmotorpid;
  private SparkMaxPIDController Rmotorpid;

  public boolean elevatorreset = false;


  public void elevatorsetup() {
    //Lmotor Setup
    Lmotor = new CANSparkMax(Constants.ElevatorConstants.LMotorID, MotorType.kBrushless);
    Lmotor.restoreFactoryDefaults();
    Lmotor.setIdleMode(IdleMode.kBrake);
    Lmotor.setSmartCurrentLimit(40);
    Lmotor.setOpenLoopRampRate(0.5);
    Lmotor.setClosedLoopRampRate(0.5);
    LmotorEncoder = Lmotor.getEncoder();
    Lmotorpid = Lmotor.getPIDController();

    //Rmotor Setup
    Rmotor = new CANSparkMax(Constants.ElevatorConstants.RMotorID, MotorType.kBrushless);
    Rmotor.restoreFactoryDefaults();
    Rmotor.setIdleMode(IdleMode.kBrake);
    Rmotor.setSmartCurrentLimit(40);
    Rmotor.setOpenLoopRampRate(0.5);
    Rmotor.setClosedLoopRampRate(0.5);
    RmotorEncoder = Rmotor.getEncoder();
    Rmotorpid = Rmotor.getPIDController();
  }

  public void resetEncoders(){
    LmotorEncoder.setPosition(0);
    RmotorEncoder.setPosition(0);
  }

public void setElevator(double value) {
    Lmotorpid.setReference(value, ControlType.kPosition, 0);
    Rmotorpid.setReference(value, ControlType.kPosition, 0);

  }

public void stopElevator(){
    Lmotor.set(0);
    Rmotor.set(0);
}

public void disableElevatorLimits() {
    Lmotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    Lmotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    Rmotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    Rmotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

public void enableElevatorLimits() {
    Lmotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    Lmotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    Rmotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    Rmotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

public void resetElevator(){
    elevatorreset = true;
    LmotorEncoder.setPosition(0);
    RmotorEncoder.setPosition(0);
  }

public double[] getElevatorCurrent(){
    double outputcurrent [] = new double[2];
    outputcurrent[0] = Lmotor.getOutputCurrent();
    outputcurrent[1] = Rmotor.getOutputCurrent();
  return outputcurrent;
  }

public void setElevatorSpeed(double value){
    Lmotor.set(value);
    Rmotor.set(value);
  }

public Elevator(){
    elevatorsetup();
  }

@Override
public void periodic() {
    SmartDashboard.putNumber("LmotorEncoder", LmotorEncoder.getPosition());
    SmartDashboard.putNumber("RmotorEncoder", RmotorEncoder.getPosition());
  }
}