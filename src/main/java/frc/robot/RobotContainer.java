package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.claw.*;
import frc.robot.commands.limelight.*;
import frc.robot.commands.nodescoring.*;
import frc.robot.commands.nodescoring.armscoring.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /* Controllers */
  private final XboxController driver = new XboxController(0);
  private final PS4Controller operator = new PS4Controller(1);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kRightY.value;
  private final int strafeAxis = XboxController.Axis.kRightX.value;
  private final int rotationAxis = XboxController.Axis.kLeftX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(
    driver,
    XboxController.Button.kY.value
  );
  private final JoystickButton music = new JoystickButton(
    driver,
    XboxController.Button.kX.value
  );
  private final JoystickButton robotCentric = new JoystickButton(
    driver,
    XboxController.Button.kLeftBumper.value
  );
  private final JoystickButton rightBumper = new JoystickButton(
    driver,
    XboxController.Button.kRightBumper.value
  );
  private final JoystickButton LTModifer = new JoystickButton(
    driver,
    XboxController.Button.kLeftStick.value
  );
  private final JoystickButton DResetArm = new JoystickButton(
    driver,
    XboxController.Button.kStart.value
  );

  /* Operator Buttons */
  private final JoystickButton LModifer = new JoystickButton(
    operator,
    PS4Controller.Button.kL2.value
  );
  private final JoystickButton RModifer = new JoystickButton(
    operator,
    PS4Controller.Button.kR2.value
  );
  private final JoystickButton bottomNode = new JoystickButton(
    operator,
    PS4Controller.Button.kCross.value
  );
  private final JoystickButton midNode = new JoystickButton(
    operator,
    PS4Controller.Button.kCircle.value
  );
  private final JoystickButton topNode = new JoystickButton(
    operator,
    PS4Controller.Button.kTriangle.value
  );
  private final JoystickButton hp = new JoystickButton(
    operator,
    PS4Controller.Button.kSquare.value
  );

  private final JoystickButton upclaw = new JoystickButton(
    operator,
    PS4Controller.Button.kShare.value
  );
  private final JoystickButton downclaw = new JoystickButton(
    operator,
    PS4Controller.Button.kOptions.value
  );
  private final JoystickButton openClaw = new JoystickButton(
    operator,
    PS4Controller.Button.kL1.value
  );
  private final JoystickButton closeClaw = new JoystickButton(
    operator,
    PS4Controller.Button.kR1.value
  );
  private final JoystickButton OResetArm = new JoystickButton(
    operator,
    PS4Controller.Button.kPS.value
  );

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Limelight limelight = new Limelight();
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  private final ClawPivot pivot = new ClawPivot();
  private final Claw claw = new Claw();

  /* autos */
  private final OnePiece onePiece = new OnePiece(s_Swerve, true);
  private final TwoPiece twoPiece = new TwoPiece(s_Swerve, false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve,
        () -> driver.getRawAxis(translationAxis),
        () -> driver.getRawAxis(strafeAxis),
        () -> driver.getRawAxis(rotationAxis),
        robotCentric::getAsBoolean
      )
    );

    // Configure the button bindings
    configureButtonBindings();

    m_chooser.setDefaultOption("OnePiece", onePiece.followPath());
    m_chooser.addOption("TwoPiece", twoPiece.followPath());

    SmartDashboard.putData(m_chooser);
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
    rightBumper.whileTrue(new AlignLimeLight(s_Swerve, limelight));

    /* Operator Buttons */
    LModifer.and(bottomNode).whileTrue(new BottomNode(elevator));
    LModifer.and(midNode).whileTrue(new MidNode(elevator));
    LModifer.and(topNode).whileTrue(new TopNode(elevator));
    LModifer.and(hp).whileTrue(new HPNode(elevator));

    RModifer.and(bottomNode).whileTrue(new BottomExtend(arm));
    RModifer.and(midNode).whileTrue(new MidExtend(arm));
    RModifer.and(topNode).whileTrue(new TopExtend(arm));

    openClaw.whileTrue(new openClaw(claw));
    closeClaw.whileTrue(new closeClaw(claw));

    upclaw.whileTrue(new pivotup(pivot));
    downclaw.whileTrue(new pivotdown(pivot));

    /* Conjoined Buttons */
    LTModifer
      .and(DResetArm.and(RModifer.and(OResetArm)))
      .whileTrue(new RunArm(arm))
      .whileFalse(new StopArm(arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
