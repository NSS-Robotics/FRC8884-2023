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
    Lmotor =
      new CANSparkMax(
        Constants.ElevatorConstants.LMotorID,
        MotorType.kBrushless
      );
    Lmotor.restoreFactoryDefaults();
    Lmotor.setIdleMode(IdleMode.kBrake);
    Lmotor.setSmartCurrentLimit(40);
    Lmotor.setOpenLoopRampRate(0.5);
    Lmotor.setClosedLoopRampRate(0.5);
    LmotorEncoder = Lmotor.getEncoder();
    Lmotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    Lmotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    Lmotor.setSoftLimit(
      SoftLimitDirection.kForward,
      Constants.ElevatorConstants.MaxHeight
    );
    Lmotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    Lmotor.setCANTimeout(0);
    //Rmotor Setup
    Rmotor =
      new CANSparkMax(
        Constants.ElevatorConstants.RMotorID,
        MotorType.kBrushless
      );
    Rmotor.restoreFactoryDefaults();
    Rmotor.setIdleMode(IdleMode.kBrake);
    Rmotor.setSmartCurrentLimit(40);
    Rmotor.setOpenLoopRampRate(0.5);
    Rmotor.setClosedLoopRampRate(0.5);
    RmotorEncoder = Rmotor.getEncoder();
    Rmotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    Rmotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    Rmotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    Rmotor.setSoftLimit(
      SoftLimitDirection.kReverse,
      -Constants.ElevatorConstants.MaxHeight
    );
    Rmotor.setCANTimeout(0);
    Rmotor.follow(Lmotor);
    resetEncoders();

    Lmotor.burnFlash();
    Rmotor.burnFlash();
  }

  public void resetEncoders() {
    LmotorEncoder.setPosition(0);
    RmotorEncoder.setPosition(0);
  }

  public void setElevator(double value) {
    Lmotorpid.setReference(value, ControlType.kPosition, 0);
    //try negative
    Rmotorpid.setReference(value, ControlType.kPosition, 0);
  }

  public void stopElevator() {
    Lmotor.set(0);
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

  public void resetElevator() {
    elevatorreset = true;
    LmotorEncoder.setPosition(0);
    RmotorEncoder.setPosition(0);
  }

  public double[] getElevatorCurrent() {
    double outputcurrent[] = new double[2];
    outputcurrent[0] = Lmotor.getOutputCurrent();
    outputcurrent[1] = Rmotor.getOutputCurrent();
    return outputcurrent;
  }

  public double[] getElevatorEncoder() {
    double outputencoder[] = new double[2];
    outputencoder[0] = LmotorEncoder.getPosition();
    outputencoder[1] = RmotorEncoder.getPosition();
    return outputencoder;
  }

  public void setElevatorSpeed(double value) {
    Lmotor.set(value);
    Rmotor.follow(Lmotor);
  }

  public Elevator() {
    elevatorsetup();
  }

  @Override
  public void periodic() {
  //   SmartDashboard.putNumber("LmotorEncoder", LmotorEncoder.getPosition());
  //   SmartDashboard.putNumber("RmotorEncoder", RmotorEncoder.getPosition());
  //   if (
  //     getElevatorEncoder()[0] >
  //     Constants.ElevatorConstants.MidNodeDistance -
  //     0.2 &&
  //     getElevatorEncoder()[1] >
  //     Constants.ElevatorConstants.MidNodeDistance -
  //     0.2 &&
  //     getElevatorEncoder()[0] <
  //     Constants.ElevatorConstants.MidNodeDistance +
  //     0.2 &&
  //     getElevatorEncoder()[1] <
  //     Constants.ElevatorConstants.MidNodeDistance +
  //     0.2
  //   ) {
  //     stopElevator();
  //   }
  //   if (
  //     getElevatorEncoder()[0] >= Constants.ElevatorConstants.TopNodeDistance &&
  //     getElevatorEncoder()[1] >= Constants.ElevatorConstants.TopNodeDistance
  //   ) {
  //     stopElevator();
  //   }
  //   if (
  //     getElevatorEncoder()[0] <=
  //     Constants.ElevatorConstants.BottomNodeDistance &&
  //     getElevatorEncoder()[1] <= Constants.ElevatorConstants.BottomNodeDistance
  //   ) {
  //     stopElevator();
  //   }
  }
}
