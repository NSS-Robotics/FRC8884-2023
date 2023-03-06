package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BoomBox extends SubsystemBase {
  Orchestra orchestra;

  public BoomBox(String song) {
    int[] motors = {
      Constants.Swerve.Mod0.driveMotorID,
      Constants.Swerve.Mod0.angleMotorID,
      Constants.Swerve.Mod1.driveMotorID,
      Constants.Swerve.Mod1.angleMotorID,
      Constants.Swerve.Mod2.driveMotorID,
      Constants.Swerve.Mod2.angleMotorID,
      Constants.Swerve.Mod3.driveMotorID,
      Constants.Swerve.Mod3.angleMotorID,
    };
    
    ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();
    for (int motor : motors) {
      instruments.add(new TalonFX(motor));
    }

    orchestra = new Orchestra(instruments);
    orchestra.loadMusic(song);

    System.out.println("Song selected is: " + song);
  }

  public void play() {
    orchestra.play();
  }

  public void pause() {
    orchestra.pause();
  }
}
