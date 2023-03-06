package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.AlignLimeLight;
<<<<<<< HEAD
//import frc.robot.commands.ArmLengths.*;
=======
// import frc.robot.commands.ArmLengths.*;
>>>>>>> c50409ba5e9c105c8e0a383b8567b933ac7c0d55
import frc.robot.commands.nodescoring.*;
import frc.robot.subsystems.*;
import frc.robot.commands.Claw.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kRightY.value;
  private final int strafeAxis = XboxController.Axis.kRightX.value;
  private final int rotationAxis = XboxController.Axis.kLeftX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(
    driver,
    XboxController.Button.kY.value
  );
  private final JoystickButton robotCentric = new JoystickButton(
    driver,
    XboxController.Button.kLeftBumper.value
  );
  private final JoystickButton rightBumper = new JoystickButton(
    driver,
    XboxController.Button.kRightBumper.value
  );
  private final JoystickButton music = new JoystickButton(
    driver,
    XboxController.Button.kX.value
  );

  private final JoystickButton bottomNode = new JoystickButton(
    operator,
    XboxController.Button.kA.value
  );
  private final JoystickButton midNode = new JoystickButton(
    operator,
    XboxController.Button.kB.value
  );
  private final JoystickButton topNode = new JoystickButton(
    operator,
    XboxController.Button.kY.value
  );

  // private final JoystickButton up = new JoystickButton(
  //   operator,
  //   XboxController.Button.kA.value
  // );
  // private final JoystickButton down = new JoystickButton(
  //   operator,
  //   XboxController.Button.kB.value
  // );

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Limelight limelight = new Limelight();
  private final Elevator elevator = new Elevator();
  private final Claw claw = new Claw();

  private final BoomBox boombox = new BoomBox("kv545.chrp");
  //private final Arm arm = new Arm();

  // private final RunMotor runmotor = new RunMotor();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            robotCentric::getAsBoolean));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(s_Swerve::zeroGyro));
    rightBumper.whileTrue(new AlignLimeLight(0, s_Swerve, limelight));
    music.whileTrue(new InstantCommand(boombox::play));
    // up
    //   .onTrue(new InstantCommand(runmotor::Extend))
    //   .onFalse(new InstantCommand(runmotor::Stop));
    // down
    //   .onTrue(new InstantCommand(runmotor::Retract))
    //   .onFalse(new InstantCommand(runmotor::Stop));
    /*
    bottomNode.whileTrue(
      new ParallelCommandGroup(
        new BottomNode(elevator)
        //new BottomExtend(arm))
      )
    );
    midNode.whileTrue(
      new ParallelCommandGroup(
        new MidNode(elevator)
        //new MidExtend(arm))
      )
    );
    topNode.whileTrue(
      new ParallelCommandGroup(
        new TopNode(elevator)
        //new TopExtend(arm);
      )
    );
    */
    bottomNode.whileTrue(new BottomNode(elevator));
    midNode.whileTrue(new MidNode(elevator));
    topNode.whileTrue(new TopNode(elevator));

    //openClaw.whileTrue(new openClaw(claw));
    //closeClaw.whileTrue(new closeClaw(claw));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new One_Piece(s_Swerve);
  }
}
