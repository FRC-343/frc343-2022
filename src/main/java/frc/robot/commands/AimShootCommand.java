package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AimShootCommand extends CommandBase {

    private final Vision m_vision;
    private final Hood m_hood;
    private final Turret m_turret;
    private final Shooter m_shooter;
    private final Kicker m_kicker;

    public AimShootCommand(Vision vision, Hood hood, Turret turret, Shooter shooter, Kicker kicker) {
        m_vision = vision;
        m_hood = hood;
        m_turret = turret;
        m_shooter = shooter;
        m_kicker = kicker;
        addRequirements(m_vision, m_hood, m_turret, m_shooter, m_kicker);
    }
}
