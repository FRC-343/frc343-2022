package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.ShootingRelatingCommands.ShootCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

public class KickerCommand extends CommandBase {

    private final Kicker m_kicker;
    public static double activateShooter[] = { 0, 0 }; // bottom speed, top speed

    public KickerCommand() {
        m_kicker = Kicker.getInstance();
        addRequirements(m_kicker);
    }

    @Override
    public void execute() {
        if (ShootCommand.activateKicker != 0) {
            m_kicker.setKicker(ShootCommand.activateKicker);
        } else if (runKickerForIntake()) {
            kickerForIntake();
        } else {
            m_kicker.setKicker(0);;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void shoot(double bottom, double top) {
        activateShooter[0] = bottom;
        activateShooter[1] = top;

    }

    private void kickerForIntake() {
        if (!m_kicker.getCellDetector()) { // if no ball is in chamber run the kicker so it goes into chanber // room for the 2nd ball in the hopper
            shoot(0, 0);
            m_kicker.setKicker(1.0);
        } else { // if getCellDetector()
            if (Robot.kUseColorSensor) {
                if (!m_kicker.isBadCargo()) { // if good cargo stop kicker
                    m_kicker.setKicker(1.0);
                    shoot(0, 0);
                } else if (m_kicker.isBadCargo()) { // if bad then shoot out
                    shoot(50, -95);
                    m_kicker.setKicker(1.0);
                }
            } else {
                m_kicker.setKicker(0.0);
                shoot(0, 0);
            }
        }

    }

    private boolean runKickerForIntake() {
        return Intake.isRunning();
        //TODO add a timer so it keeps running for a little longer after intake
    }

}
