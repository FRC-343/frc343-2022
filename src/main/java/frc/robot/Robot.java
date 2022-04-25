package frc.robot;

import frc.robot.autonomous.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.commands.ShootingRelatingCommands.*;
import frc.robot.commands.ShootingRelatingCommands.SpecificCommands.*;
import frc.robot.utils.MiscMath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Button;

import edu.wpi.first.wpilibj.Compressor;

public class Robot extends TimedRobot {
    public static final double kMaxJoySpeed = 3.0; // meters per sec
    public static final double kMaxJoyTurn = 5.0; // radians per sec
    public static final double kMaxHoodSpeed = 1.0; // ratio
    public static final double kMaxWinchSpeed = 1.0;
    public static final double kMaxTurretSpeed = 0.6;
    public static final double kMaxClimbingSpeed = .8;

    public final static boolean kUseColorSensor = false;
    public final static boolean kUseColorSensorIntake = true;

    private static final Compressor Pressy = new Compressor(0, PneumaticsModuleType.CTREPCM);

    private final Drive m_drive = Drive.getInstance();
    private final Hood m_hood = Hood.getInstance();
    private final Shooter m_shooter = Shooter.getInstance();
    private final Vision m_vision = Vision.getInstance();
    private final Turret m_turret = Turret.getInstance();
    private final Kicker m_kicker = Kicker.getInstance();
    private final Intake m_intake = Intake.getInstance();
    private final Climbing m_climbing = Climbing.getInstance();

    private final XboxController m_controller = new XboxController(1);
    private final Joystick m_stick = new Joystick(0);

    private CommandBase m_auto;
    private final SendableChooser<CommandBase> m_autoChooser = new SendableChooser<CommandBase>();

    public Robot() {
        m_autoChooser.setDefaultOption("No_Auto", new NoAutonomous());
        m_autoChooser.addOption("2BA", new TwoBallAuto());
        m_autoChooser.addOption("3BA", new ThreeBallAuto());
        m_autoChooser.addOption("Simple", new MoveAuto());
        m_autoChooser.addOption("T_5", new CCW5ball2022());
        m_auto = m_autoChooser.getSelected();

    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

        SmartDashboard.putData("Auto_Choice", m_autoChooser);

        Pressy.enableDigital(); // compressor has to be enabled manually

        // Background commands that activate themselves when ready
        m_shooter.setDefaultCommand(new ShooterCommand());
        m_kicker.setDefaultCommand(new KickerCommand());

        // Joystick
        m_drive.setDefaultCommand(new RunCommand(() -> m_drive.drive(kMaxJoySpeed *
                MiscMath.deadband(-m_stick.getY()),
                kMaxJoyTurn * MiscMath.deadband(-m_stick.getX())), m_drive));

        // Joystick buttons

        new JoystickButton(m_stick, 10).whenPressed(new InstantCommand(m_intake::lower, m_intake));
        new JoystickButton(m_stick, 11).whenPressed(new InstantCommand(m_intake::raise, m_intake));

        // Joystick Trigger

        new JoystickButton(m_stick, 1).whenHeld(new IntakeCommand(.8))
                .whenReleased(new Intake2Command(.8));

        // Other Joystick Buttons (turret Presets)

        new JoystickButton(m_stick, 3).whenPressed(new PresetTurretCommand(110));

        new JoystickButton(m_stick, 4).whenHeld(new RunCommand(() -> {
            m_turret.spin(-kMaxTurretSpeed);
        }, m_turret)).whenReleased(new RunCommand(() -> {
            m_turret.stop();
        }, m_turret));

        new JoystickButton(m_stick, 5).whenHeld(new RunCommand(() -> {
            m_turret.spin(kMaxTurretSpeed);
        }, m_turret)).whenReleased(new RunCommand(() -> {
            m_turret.stop();
        }, m_turret));

        // Controller joysticks
        m_hood.setDefaultCommand(
                new RunCommand(() -> m_hood.move(kMaxHoodSpeed * m_controller.getRightY()), m_hood));

        m_turret.setDefaultCommand(
                new RunCommand(() -> m_turret.spin(kMaxTurretSpeed * m_controller.getRightX()), m_turret));

        m_climbing.setDefaultCommand(
                new RunCommand(() -> m_climbing.setWinch(kMaxClimbingSpeed * m_controller.getLeftY()), m_climbing));

        // Controller Triggers/Bumpers

        new Button(() -> m_controller.getRightTriggerAxis() > 0.2).whenHeld(new AimShootCommand()); // shooter

        new Button(() -> m_controller.getRightBumper())
                .whenHeld(new SequentialCommandGroup(new InstantCommand(ShootCommand::useLowGoal),
                        new ShootCommand())); // low goal

        new Button(() -> m_controller.getLeftBumper()).whenHeld(new AimShootMoveCommand()); // low goal

        new Button(() -> m_controller.getLeftTriggerAxis() > 0.2).whenHeld(new IntakeCommand(.8))
                .whenReleased(new Intake2Command(.8));

        // Controller Buttons
        new Button(() -> m_controller.getYButton()).whenHeld(new IntakeCommand(-.3));

        new JoystickButton(m_controller, XboxController.Button.kA.value).whenPressed(new RunCommand(() -> {
            m_kicker.setKicker(1.0);
        }, m_kicker)).whenReleased(new RunCommand(() -> {
            m_kicker.setKicker(0.0);
        }, m_kicker));

        new JoystickButton(m_controller, XboxController.Button.kB.value).whenPressed(new RunCommand(() -> {
            m_kicker.setKicker(-1.0);
        }, m_kicker)).whenReleased(new RunCommand(() -> {
            m_kicker.setKicker(0.0);
        }, m_kicker));

        new JoystickButton(m_controller, XboxController.Button.kX.value).whenPressed(new PresetHoodCommand(0, true));

        new JoystickButton(m_controller, XboxController.Button.kBack.value)
                .whenPressed(new InstantCommand(m_climbing::toBeOrNotToBe, m_climbing)); // toggle climber pnumatics

        new JoystickButton(m_controller, XboxController.Button.kStart.value)
                .whenHeld(new AutoClimbCommand(false, false)); // climbing auto
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_vision.setLEDS(true);
        m_drive.zeroHeading();

        m_auto = m_autoChooser.getSelected();

        if (m_auto != null) {
            m_auto.schedule();
        }

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    /**
     * This function is called when entering operator control.
     */
    @Override
    public void teleopInit() {
        if (m_auto != null) {
            m_auto.cancel();
        }

        m_drive.zeroHeading();
        m_vision.setLEDS(true);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledInit() {
        m_vision.setLEDS(false);
    }

}
