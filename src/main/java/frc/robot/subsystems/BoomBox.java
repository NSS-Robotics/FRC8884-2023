package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BoomBox extends SubsystemBase {
    Orchestra orchestra;
    
    public BoomBox(String song) {
        int[] ids = {4, 3, 6, 16, 2, 5, 7, 15};
        
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