package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;

public class ShooterCommand extends CommandBase {

    private final Shooter m_shooter;

    public ShooterCommand(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        if (ShootCommand.activateShooter[0] != 0 || ShootCommand.activateShooter[1] != 0) {
            m_shooter.shoot(ShootCommand.activateShooter[0], ShootCommand.activateShooter[1]);
        } else if (IntakeCommand.activateShooter[0] != 0 || IntakeCommand.activateShooter[1] != 0) {
            m_shooter.shoot(IntakeCommand.activateShooter[0], IntakeCommand.activateShooter[1]);
        } else {
            m_shooter.shoot(0);
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
