package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AimCommand extends CommandBase {

    private final Vision m_vision;
    private final Hood m_hood;
    private final Turret m_turret;

    private double kTurretPrecision = 2.0;
    private double kTurretSpeed = .3;

    private boolean m_stop;

    public static double kShooterSpeedFromAim = -1.0;

    public AimCommand(Vision vision, Hood hooooooooood, Turret turret, boolean stop) {
        m_vision = vision;
        m_hood = hooooooooood;
        m_turret = turret;
        m_stop = stop;
        addRequirements(m_vision, m_hood, m_turret);
    }

    public AimCommand(Vision vision, Hood hood, Turret turret) {
        this(vision, hood, turret, true);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // m_vision.setCamera(2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double heading_error = m_vision.getTx();
        double x = m_vision.getTy();

        kTurretSpeed = Math.abs(heading_error) / 30.0; // equivilent to a PID, goes proportionally slower the closer you are
        if (kTurretSpeed > .4) { // increase these to 5 if it doesn't break
            kTurretSpeed = .4;
        } else if (kTurretSpeed < .17) {
            kTurretSpeed = .17;
        }

        if (heading_error > kTurretPrecision) {
            m_turret.spin(kTurretSpeed);
        } else if (heading_error < -kTurretPrecision) {
            m_turret.spin(-kTurretSpeed);
        } else {
            m_turret.spin(0.0);
        }

        if (x < 18 && x > -.5) {
            m_hood.aim(866.4648 + x * -52.5726);
        } else { // if (x <= -
            m_hood.aim(579.835 + x * -185.9704);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_hood.move(0.0);
        m_turret.spin(0.0);

        double x = m_vision.getTy();

        if (x < 18 && x > -.5) {
            kShooterSpeedFromAim = 70;
        } else { // if (x <= -
            kShooterSpeedFromAim = 75;
        }

        System.out.println("IT is over");

        // m_vision.setCamera(1);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_stop) {
            return m_hood.isAimed() && (Math.abs(m_vision.getTx()) < kTurretPrecision);
        } else {
            return false;
        }
    }

}
