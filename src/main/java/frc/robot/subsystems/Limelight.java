package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Limelight extends SubsystemBase {
    

    public double Kp = -0.1;
    public double min_command = 0.05;

    public double tv = 0;
    public double tx = 0;
    public double ty = 0;
    public double ta = 0;
    public double dist = 0;
    public Swerve swerve = new Swerve();

    public void turnOffLimelight() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public void turnOnLimelight() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public double estimateDistance() {
        double targetOffsetAngle_Vertical = ty;
        double angletoGoalRad = (Constants.MountAngle + 
                                targetOffsetAngle_Vertical) * 
                                (Math.PI / 180.0);
        return (Constants.TargetHeight - Constants.MountHeight) / 
                Math.tan(angletoGoalRad);

    }

    public void updateLimelightTracking() {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightV", tv);
        SmartDashboard.putNumber("LimelightA", ta);
    }

    public double gettx() {
        updateLimelightTracking();
        return tx;
    }
    public boolean hasValidTarget() {
        if (tv < 1.0) {
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void periodic() {
        updateLimelightTracking();

        SmartDashboard.putNumber("Dist to Target", estimateDistance());

        
        
    }

}