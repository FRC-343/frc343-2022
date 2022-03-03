package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Turret;

public class PresetTurretCommand extends CommandBase {

    private final Turret m_turret;

    private double kTarget;
    
    public PresetTurretCommand(Turret turret, double target) {
        m_turret = turret;
        kTarget = target;
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        m_turret.aim(kTarget);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }

    @Override
    public boolean isFinished() {
        return m_turret.isAimed();
    }

}