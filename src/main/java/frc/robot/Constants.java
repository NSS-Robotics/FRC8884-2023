package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import java.util.HashMap;

public final class Constants {

  public static final double stickDeadband = 0.1;

  /* limelight constants */
  public static final double mountAngle = 0.0; // angle from 90 vertical
  public static final double mountHeight = 67.0; // Mount height in cm from ground
  public static final double mountOffset = 24.0; // horizontal limelight mount offset in cm

  public static final double distToScoring = 50.0; // the distance for the robot to stop at before the scoring station
  public static final double turnTolerance = 2.0; // degrees per turn

  public static final double tagHeight = 36.0; // cube apriltag mount height in cm
  public static final double tapeHeight = 56.0; // cone reflective tape mount height in cm
  public static final double playerStationHeight = 91.0; // human player station april tag mount height in cm

  /* Limelight Align PID */
  public static final double kTurnP = 0.1;
  public static final double kTurnI = 0.0;
  public static final double kTurnD = 0.0;

  /* Limelight Lateral PID */
  public static final double kLateralP = 3;
  public static final double kLateralI = 0.1;
  public static final double kLateralD = 0.1;

  /* Gryo Align PID */
  public static final double kGyroP = 0.02;
  public static final double kGyroI = 0.2;
  public static final double kGyroD = 0.01;

  /* Balancing Constants */
  public static final double kLevel = 0;
  public static final double kBalanceToleranceDeg = 3.5;
  public static final double kBalanceP = .008;
  public static final double kBalanceI = 0;
  public static final double kBalanceD = 0;

  public static final double kAngleCorrectionP = .01;
  public static final double kTurnToleranceDeg = 10;
  public static final double kTurnRateToleranceDegPerS = 10;

  public static final class ArmConstants {

    public static final int MotorID = 15;
    public static final double ArmSpeed = 0.2;
    public static final int ExtendBottomNode = 9;
    public static final int ExtendMidNode = 120;
    public static final int ExtendTopNode = 240;
    public static final int ExtendMax = 240;
    public static final double Kp = 0.0451453;
    public static final double Ki = 0;
    public static final double Kd = 0.00035143;
  }

  /* Elevator Constants */
  public static final class ElevatorConstants {

    public static final int LMotorID = 12;
    public static final int RMotorID = 13;
    public static final int BottomNodeDistance = 1;
    public static final int MidNodeDistance = 58;
    public static final double HPDistance = 55;
    public static final int TopNodeDistance = 73;
    public static final int MaxHeight = 72;
    public static final double Kp = 0.0222244;
    public static final double Ki = 0;
    public static final double Kd = 0.00063787;
  }

  public static final class Swerve {

    public static final int pigeonID = 1;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK4i(
      COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2
    );

    /* Drivetrain Constants */
    public static final double trackWidth = 0.712;
    public static final double wheelBase = 0.712;
    public static final double wheelDiameter = 0.10033;
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert =
      chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert =
      chosenModule.driveMotorInvert;

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
    public static final double driveKP = 0.05;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5;
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0;

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {

      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(85);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {

      public static final int driveMotorID = 6;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 8;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(79);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {

      public static final int driveMotorID = 2;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(52);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {

      public static final int driveMotorID = 7;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(26);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }
  }

  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared =
      Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
    public static final HashMap<String, Command> eventMap = new HashMap<>();

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularSpeedRadiansPerSecondSquared
    );
  }
}
