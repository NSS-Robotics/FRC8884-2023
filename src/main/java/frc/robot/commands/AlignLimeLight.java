package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Limelight;
import java.util.function.DoubleSupplier;


import frc.robot.subsystems.Swerve;

/*public class AlignLimeLight extends PIDCommand {

    private final Swerve swerve;
    

    //TODO: Fix this constructor shit
    //DriveSubsystem drive;
    
    public AlignLimelight(Swerve s_Swerve) {
            
            super(
            new PIDController(Constants.turn_P, 
                Constants.turn_I, 
                Constants.turn_D),

            Limelight.gettx(), 

            0,

            (s_Swerve.drive(placeholder1, placeholder2, placeholder3, placeholder4), //placeholder shit
            
            
            s_Swerve);
    
            

    // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController()
            .setTolerance(Constants.turnTolerance);
            System.out.println("Align With Limelight - Start");
            swerve = s_Swerve;


        } 


    
    @Override
    public boolean isFinished() {
    // End when the controller is at the reference.
      return getController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
      
        //swerve.drive(0, 0, false, false);
        System.out.println("Align With Limelight - End");
    }
}*/

/** A command that will turn the robot to the specified angle. */

