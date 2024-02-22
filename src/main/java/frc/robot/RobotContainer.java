package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton shooter = new JoystickButton(operator, 1);
    private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton bottomLeftJoystick = new JoystickButton(operator, 3);
    private final JoystickButton topRightJoystick = new JoystickButton(operator, 6);
    private final JoystickButton topLeftJoystick = new JoystickButton(operator, 5);
    private final JoystickButton sideButton = new JoystickButton(operator, 2);
    private final JoystickButton bottomRightJoystick = new JoystickButton(operator, 4);
    private final JoystickButton button7 = new JoystickButton(operator, 7);
    //private final JoystickButton midRight = new JoystickButton(operator, 10);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();
    private final Climber s_Climber = new Climber();

    // commands 
    Command shoot = new Shoot(s_Intake,s_Swerve, () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis));
    Command followNote = new FollowNote(s_Swerve, s_Intake,() -> -driver.getRawAxis(translationAxis));
    SequentialCommandGroup autoAmp = new SequentialCommandGroup();
    // autos 
    private String mainAuto = "Main";
    private String backupAuto = "BackupAuto";

    SendableChooser<String> m_chooser = new SendableChooser<>();
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        m_chooser.setDefaultOption("Main Auto", mainAuto);
        m_chooser.addOption("Backup Auto", backupAuto);
        SmartDashboard.putData(m_chooser);
        // Configure the button bindings
        //s_Intake.setDefaultCommand(new InstantCommand(() -> s_Intake.intake(), s_Intake));
        configureButtonBindings();
        autoAmp.addCommands(new InstantCommand(() -> s_Intake.setIntakeMode(IntakeMode.amp)));
        autoAmp.addCommands(new Shoot(s_Intake, s_Swerve, () -> 0, () -> 0));
        autoAmp.addCommands(new InstantCommand(() -> s_Intake.setIntakeMode(IntakeMode.shooter)) );
        NamedCommands.registerCommand("shooterMode", new InstantCommand(() -> s_Intake.setIntakeMode(IntakeMode.shooter)));
        NamedCommands.registerCommand("intakeMode", new InstantCommand(() -> s_Intake.setIntakeMode(IntakeMode.normal)));

        NamedCommands.registerCommand("shoot", new Shoot(s_Intake, s_Swerve, () -> 0, () -> 0));
        NamedCommands.registerCommand("pickup", new FollowNote(s_Swerve, s_Intake,() -> -1).withTimeout(1.2));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        aButton.onTrue(new InstantCommand(() -> s_Climber.climberUp()));
        bButton.onTrue(new InstantCommand(() -> s_Climber.climberDown()));
        aButton.onFalse(new InstantCommand(() -> s_Climber.climberStop()));
        bButton.onFalse(new InstantCommand(() -> s_Climber.climberStop()));

        bottomLeftJoystick.onTrue(new InstantCommand(() -> s_Intake.setIntakeMode(IntakeMode.normal)));
        topLeftJoystick.onTrue(new InstantCommand(() -> s_Intake.setIntakeMode(IntakeMode.shooter)));
        bottomRightJoystick.onTrue(new InstantCommand(() -> s_Intake.setIntakeMode(IntakeMode.amp)));
        topRightJoystick.onTrue(new InstantCommand(() -> s_Intake.setIntakeMode(IntakeMode.halt)));

        sideButton.onTrue(new InstantCommand(() -> s_Intake.toggleIntake()));
        shooter.whileTrue(shoot);
        rightBumper.whileTrue(followNote);
        button7.onTrue(autoAmp);
        //rightBumper.onTrue(new InstantCommand(() -> s_Intake.))
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new PathPlannerAuto(m_chooser.getSelected());
    }
}
