package frc.robot;

import frc.robot.autonomous.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Button;

import edu.wpi.first.wpilibj.Compressor;

public class Robot extends TimedRobot {
    public static final double kMaxJoySpeed = 3.0; // meters per sec
    public static final double kMaxJoyTurn = 5.0; // radians per sec
    public static final double kMaxHoodSpeed = 1.0; // ratio
    public static final double kMaxWinchSpeed = 1.0; // ratio
    public static final double kMaxTurretSpeed = 0.5; // ratio

    public static final double kTargetP = -0.055;
    public static final double kMinTargetCommand = -0.35;

    private static final Compressor Pressy = new Compressor(0, PneumaticsModuleType.CTREPCM);

    private final Drive m_drive = new Drive();
    private final Hood m_hood = new Hood();
    private final Shooter m_shooter = new Shooter();
    private final Vision m_vision = new Vision();
    private final Turret m_turret = new Turret();
    private final Kicker m_kicker = new Kicker();
    private final Intake m_intake = new Intake();

    // private final Climbing m_climbing = new Climbing();

    private final XboxController m_controller = new XboxController(1);
    private final Joystick m_stick = new Joystick(0);

    private CommandBase m_auto;
    private final SendableChooser<CommandBase> m_autoChooser = new SendableChooser<CommandBase>();

    public Robot() {
        m_autoChooser.setDefaultOption("No_Auto", new NoAutonomous());
        m_autoChooser.addOption("5CCWALT",
                new CCW5ball2022Alt(m_drive, m_intake, m_kicker, m_vision, m_hood, m_shooter, m_turret));
        m_autoChooser.addOption("3CCW",
                new CCW3ball2022(m_drive, m_intake, m_kicker, m_vision, m_hood, m_shooter, m_turret));
        // m_autoChooser.addOption("BIAS", new JustBackItUpAndShoot(m_drive, m_intake,
        // m_kicker, m_vision, m_hood, m_shooter));
        // m_auto = m_autoChooser.getSelected();

    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

        SmartDashboard.putData("Auto_Choice", m_autoChooser);

        Pressy.enableDigital(); // compressor has to be enabled manually

        // m_climbing.setDefaultCommand(new RunCommand(
        // () -> m_climbing.setWinch(-kMaxWinchSpeed * m_controller.getRightY()),
        // m_climbing));

        // m_drive.setDefaultCommand(new RunCommand(() -> m_drive.drive(kMaxJoySpeed *
        // MiscMath.deadband(-m_stick.getY()),
        // kMaxJoyTurn * MiscMath.deadband(-m_stick.getX())), m_drive));

        m_drive.setDefaultCommand(new RunCommand(
                () -> m_drive.setVoltages(12 * MiscMath.deadband(-m_stick.getY() + m_stick.getX()),
                        12 * MiscMath.deadband(-m_stick.getY() - m_stick.getX())),
                m_drive));

        m_hood.setDefaultCommand(
                new RunCommand(() -> m_hood.move(kMaxHoodSpeed * m_controller.getLeftY()), m_hood));

        m_turret.setDefaultCommand(
                new RunCommand(() -> m_turret.spin(kMaxTurretSpeed * m_controller.getRightX()), m_turret));

        m_shooter.setDefaultCommand(
                new RunCommand(() -> m_shooter.set(0.35 * m_controller.getRightTriggerAxis(),
                        0.70 * m_controller.getRightTriggerAxis()), m_shooter));

        new JoystickButton(m_controller, XboxController.Button.kA.value).whenPressed(new RunCommand(() -> {
            m_kicker.setKicker(1.0);
        }, m_kicker)).whenReleased(new RunCommand(() -> {
            m_kicker.setKicker(0.0);
        }, m_kicker));

        new JoystickButton(m_controller, XboxController.Button.kY.value).whenPressed(new RunCommand(() -> {
            m_intake.setIntake(-0.3);
        }, m_intake)).whenReleased(new RunCommand(() -> {
            m_intake.setIntake(0);
        }, m_intake));

        new JoystickButton(m_stick, 11).whenPressed(new InstantCommand(m_intake::raise, m_intake));
        new JoystickButton(m_stick, 10).whenPressed(new InstantCommand(m_intake::lower, m_intake));
        new JoystickButton(m_stick, 9).whenHeld(new AimCommand(m_vision, m_hood, m_turret));

        new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value)
                .whenHeld(new ShootCommand(m_shooter, m_kicker));
        new JoystickButton(m_controller, XboxController.Button.kRightBumper.value)
                .whenHeld(new AimShootCommand(m_vision, m_hood, m_turret, m_shooter, m_kicker));

        new Button(() -> m_controller.getLeftTriggerAxis() > 0.2).whenHeld(new IntakeCommand(m_intake, m_kicker));

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
    public void testPeriodic() {

    }
}
