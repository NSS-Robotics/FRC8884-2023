package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {

  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public final AHRS gyro;

  public Swerve() {
    gyro = new AHRS(SPI.Port.kMXP);

    zeroGyro();

    mSwerveMods =
      new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants),
      };

    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry =
      new SwerveDriveOdometry(
        Constants.Swerve.swerveKinematics,
        getYaw(),
        getModulePositions()
      );
  }

  public void drive(
    Translation2d translation,
    double rotation,
    boolean fieldRelative,
    boolean isOpenLoop
  ) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(),
          translation.getY(),
          rotation,
          getYaw()
        )
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates,
      Constants.Swerve.maxSpeed
    );

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      Constants.Swerve.maxSpeed
    );

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.zeroYaw();
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
      ? Rotation2d.fromDegrees(360 - gyro.getYaw())
      : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public void turnStates(double angularSpeed) {
    var swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        angularSpeed,
        gyro.getRotation2d()
      )
    );
    setModuleStates(swerveModuleStates);
  }

  public void XFormation() {
    SwerveModuleState[] desiredstates = {
      new SwerveModuleState(1, Rotation2d.fromDegrees(40)),
      new SwerveModuleState(1, Rotation2d.fromDegrees(124)),
      new SwerveModuleState(1, Rotation2d.fromDegrees(124)),
      new SwerveModuleState(1, Rotation2d.fromDegrees(65)),
    };

    setModuleStates(desiredstates);
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());

    SmartDashboard.putString(
      "Robot Location",
      getPose().getTranslation().toString()
    );

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Cancoder",
        mod.getCanCoder().getDegrees()
      );
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Integrated",
        mod.getPosition().angle.getDegrees()
      );
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Velocity",
        mod.getState().speedMetersPerSecond
      );
    }
  }
}
