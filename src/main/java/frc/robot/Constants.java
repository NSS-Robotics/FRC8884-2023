package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static final double stickDeadband = 0.1;

  //limelight constants
  public static final double MountAngle = 90.0; //angle from 90 vertical
  public static final double MountHeight = 74.7; //Mount height in cm
  //TODO:check if this is right
  public static final double TargetHeight = 60.0; //Target height in cm
  public static final double turnTolerance = 2.0; //degrees per turn
  //Limelight PID
  public static final double turn_P = 0.1;
  public static final double turn_I = 0.0;
  public static final double turn_D = 0.0;

  /* Elevator Constants */
  public static final class ElevatorConstants {

    public static final int LMotorID = 12;
    public static final int RMotorID = 13;
    public static final int BottomNodeDistance = 0;
    public static final int MidNodeDistance = 22;
    public static final int TopNodeDistance = 48;
    public static final int MaxHeight = 48;
  }

  /* Arm Constants */
  public static final class ArmConstants {

    //TODO: check if this is right
    public static final int MotorID = 14;
    public static final int ExtendTopNode = 0;
    public static final int ExtendBottomNode = 0;
    public static final int ExtendMidNode = 0;
    public static final int ExtendMax = 0;
  }

  public static final class Swerve {

    public static final int pigeonID = 1;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =
        COTSFalconSwerveConstants.SDSMK4i( // TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = 0.712;
    public static final double wheelBase = 0.712;
    public static final double wheelDiameter = 0.10033;
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity =
        10.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot

      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(265);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot

      public static final int driveMotorID = 6;
      public static final int angleMotorID = 16;
      public static final int canCoderID = 8;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(259);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot

      public static final int driveMotorID = 2;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(232);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot

      public static final int driveMotorID = 7;
      public static final int angleMotorID = 15;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(206);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final
  class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
                        // to specific robot

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
