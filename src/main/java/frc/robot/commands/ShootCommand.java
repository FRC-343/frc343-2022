package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends CommandBase {
  private static final double kTopShootSpeed = 110.00; // rev per sec 125.0
  private static final double kTopShootReadySpeed = 100.0; // rev per sec

  private static final double kBottomShootSpeed = 110.00; // rev per sec 125.0
  private static final double kBottomShootReadySpeed = 100.0; // rev per sec

  private final Shooter m_shooter;
  private final Kicker m_kicker;

  public ShootCommand(Shooter shooter, Kicker kicker) {
    m_shooter = shooter;
    m_kicker = kicker;
    addRequirements(m_shooter, m_kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.shoot(kBottomShootSpeed, kTopShootSpeed);
    if (m_shooter.getBottomShooterRPS() >= kBottomShootReadySpeed
        && m_shooter.getTopShooterRPS() >= kTopShootReadySpeed) {
      m_kicker.setKicker(-1.0);
    } else {
      m_kicker.setKicker(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.shoot(0);
    m_kicker.setKicker(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
