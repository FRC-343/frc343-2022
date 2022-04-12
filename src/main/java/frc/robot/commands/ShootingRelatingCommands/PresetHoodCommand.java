package frc.robot.commands.ShootingRelatingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Hood;

public class PresetHoodCommand extends CommandBase {

    private final Hood m_hood;

    private boolean m_startWithZeroing;

    private double kTarget;
    
    public PresetHoodCommand(Hood hood, double target, boolean startWithZeroing) {
        m_hood = hood;
        kTarget = target;
        m_startWithZeroing = startWithZeroing;
        addRequirements(m_hood);
    }

    public PresetHoodCommand(Hood hood, double target) {
        this(hood, target, false);
    }

    @Override
    public void execute() {
        m_hood.aim(kTarget, m_startWithZeroing);
    }

    @Override
    public void end(boolean interrupted) {
        m_hood.stop();
    }

    @Override
    public boolean isFinished() {
        return m_hood.isAimed();
    }

}
