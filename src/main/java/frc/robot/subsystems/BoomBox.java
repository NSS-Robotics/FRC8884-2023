package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BoomBox extends SubsystemBase {
<<<<<<< HEAD
    Orchestra orchestra;
    
    public BoomBox(String song) {
        int[] ids = {4, 3, 6, 16, 2, 5, 7, 15};
        
        ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();
        for (int id : ids) {
            instruments.add(new TalonFX(id));
        }
=======
  Orchestra orchestra;
>>>>>>> c50409ba5e9c105c8e0a383b8567b933ac7c0d55

  public BoomBox(String song) {
    int[] ids = {4, 3, 6, 1, 2, 5, 7, 0};

    ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();
    for (int id : ids) {
      instruments.add(new TalonFX(id));
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
