package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

  public double Kp = -0.1;
  public double min_command = 0.05;

  public NetworkTable table;
  public double pipeline;

  public double tv = 0;
  public double tx = 0;
  public double ty = 0;
  public double ta = 0;
  public double dist = 0;
  public Swerve swerve = new Swerve();

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void turnLimelightLED(boolean on) {
    table.getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  public double estimateDistance() {
    double angletoGoalRad = Math.toRadians(Constants.mountAngle + getty());
    return (
      Math.abs(Constants.targetHeight - Constants.mountHeight) /
      Math.tan(angletoGoalRad)
    );
  }

  public void updateLimelightTracking() {
    tv = table.getEntry("tv").getDouble(0);
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);
    SmartDashboard.putNumber("LimelightX", tx);
    SmartDashboard.putNumber("LimelightY", ty);
    SmartDashboard.putNumber("LimelightV", tv);
    SmartDashboard.putNumber("LimelightA", ta);
  }

  public double gettx() {
    updateLimelightTracking();
    return tx;
  }

  public double getty() {
    updateLimelightTracking();
    return ty;
  }

  public boolean hasValidTarget() {
    return tv >= 1.0;
  }

  public double getPipeline() {
    pipeline = table.getEntry("getpipe").getDouble(0);
    return pipeline;
  }

  public void setPipeline(double pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
    SmartDashboard.putNumber("Pipeline", pipeline);
  }

  public double lateralDistance() {
    double hypotDist = estimateDistance();
    double distance = Math.sin(Math.toRadians(gettx())) * hypotDist;
    System.out.println("latdist: " + distance);
    return distance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RotateLimelight", -1);

    updateLimelightTracking();
    SmartDashboard.putNumber("Dist to Target", estimateDistance());
  }
}
