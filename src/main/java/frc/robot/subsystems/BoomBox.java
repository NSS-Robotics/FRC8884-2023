package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BoomBox extends SubsystemBase {
    //orchestra
    Orchestra orchestra;
    String[] songs = {"kv545.chrp"};
    
    
    public BoomBox() {
        TalonFX[] motors = {new TalonFX(2), 
            new TalonFX(3), 
            new TalonFX(4),
            new TalonFX(5), 
            new TalonFX(6), 
            new TalonFX(7), 
            new TalonFX(1), 
            new TalonFX(0)};
        
        ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();
        for (TalonFX motor : motors) {
            instruments.add(motor);
        }
        orchestra = new Orchestra(instruments);
        orchestra.loadMusic(songs[0]);
        System.out.println("Song selected is: " + songs[0]);
    }

    public void play() {
        orchestra.play();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}