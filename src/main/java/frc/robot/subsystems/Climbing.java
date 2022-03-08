package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbing extends SubsystemBase {
  private final DoubleSolenoid m_climber = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 3, 2);

  private final Spark m_leftWinch = new Spark(8); // left from when behind robot
  private final Spark m_rightWinch = new Spark(9);

  private final DigitalInput m_isLeftTop = new DigitalInput(15);
  private final DigitalInput m_isLeftBottom = new DigitalInput(16);
  private final DigitalInput m_isRightTop = new DigitalInput(17);
  private final DigitalInput m_isRightBottom = new DigitalInput(18);

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

  public void setWinch(double speed) {
    setLeftWinch(speed);
    setRightWinch(speed);
  }

  public void setLeftWinch(double speed) {
    if (speed > 0 && m_isLeftTop.get()) {
      m_leftWinch.set(0.0);
    } else if (speed < 0 && m_isLeftBottom.get()) {
      m_leftWinch.set(0.0);
    } else {
      m_leftWinch.set(speed);
    }
  }

  public void setRightWinch(double speed) {
    if (speed > 0 && m_isRightTop.get()) {
      m_rightWinch.set(0.0);
    } else if (speed < 0 && m_isRightBottom.get()) {
      m_rightWinch.set(0.0);
    } else {
      m_rightWinch.set(speed);
    }
  }

}