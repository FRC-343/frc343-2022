package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbing extends SubsystemBase {
  private final DoubleSolenoid m_climber = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 3, 2); //TODO flip values

  public Climbing() {
    SendableRegistry.setSubsystem(m_climber, this.getClass().getSimpleName());
    SendableRegistry.setName(m_climber, "climbing pnumatics");
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("climber pnumatics State", m_climber.get() == DoubleSolenoid.Value.kForward);
  }

  public void disEngage() {
    m_climber.set(DoubleSolenoid.Value.kReverse);
  }

  public void engage() {
    m_climber.set(DoubleSolenoid.Value.kForward);
  }

  public void toBeOrNotToBe() {
    if (m_climber.get() == DoubleSolenoid.Value.kReverse) {
      engage();
    } else {
      disEngage();
    }
  }
}