package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends CommandBase {
  private static final double kShootSpeed = 110.00; // rev per sec 125.0
  private static final double kShootReadySpeed = 100.0; // rev per sec

  private final Shooter m_shooter;
  private final Kicker m_kicker;
  private final BooleanSupplier m_whenToShoot;

  public ShootCommand(Shooter shooter, Kicker kicker, BooleanSupplier whenToShoot) {
    m_shooter = shooter;
    m_kicker = kicker;
    m_whenToShoot = whenToShoot;
    addRequirements(m_shooter, m_kicker);
  }
  
  public ShootCommand(Shooter shooter, Kicker kicker) {
    this(shooter, kicker, () -> true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override //TODO fix kicker logic
  public void execute() {
    m_shooter.shoot(kShootSpeed);
    if (m_whenToShoot.getAsBoolean()) {
      m_kicker.setKicker(-1.0);
      // if (m_shooter() > kShootReadySpeed) {}     
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
