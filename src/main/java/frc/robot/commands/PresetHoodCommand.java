package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Hood;

public class PresetHoodCommand extends CommandBase {

    private final Hood m_hood;

    private double kTarget;
    
    public PresetHoodCommand(Hood hood, double target) {
        m_hood = hood;
        kTarget = target;
        addRequirements(m_hood);
    }

    @Override
    public void execute() {
        m_hood.aim(kTarget);
    }

    @Override
    public void end(boolean interrupted) {
        m_hood.stop();
    }

    @Override
    public boolean isFinished() {
        return Hood.isAimed();
    }

}
