package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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

    private final DigitalInput m_isLeftTop = new DigitalInput(17);
    private final DigitalInput m_isLeftBottom = new DigitalInput(18);
    private final DigitalInput m_isRightTop = new DigitalInput(16);
    private final DigitalInput m_isRightBottom = new DigitalInput(15);

    public Climbing() {
        SendableRegistry.setSubsystem(m_climber, this.getClass().getSimpleName());
        SendableRegistry.setName(m_climber, "climbing pnumatics");

        SendableRegistry.setSubsystem(m_isLeftBottom, this.getClass().getSimpleName());
        SendableRegistry.setName(m_isLeftBottom, "isleftbottom");
        SendableRegistry.setSubsystem(m_isLeftTop, this.getClass().getSimpleName());
        SendableRegistry.setName(m_isLeftTop, "isleftTop");

        SendableRegistry.setSubsystem(m_isRightBottom, this.getClass().getSimpleName());
        SendableRegistry.setName(m_isRightBottom, "isrightbottom");
        SendableRegistry.setSubsystem(m_isRightTop, this.getClass().getSimpleName());
        SendableRegistry.setName(m_isRightTop, "isrighttop");

        m_rightWinch.setInverted(true);
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

    public boolean getLeftTopLimit() {
        return m_isLeftTop.get();
    }

    public boolean getRightTopLimit() {
        return m_isRightTop.get();
    }

    public boolean getLeftBottomLimit() {
        return m_isLeftBottom.get();
    }

    public boolean getRightBottomLimit() {
        return m_isRightBottom.get();
    }

    public void setWinch(double speed) {
        // right
        if (speed < 0.0 && (m_isRightTop.get())) {
            m_rightWinch.set(0.0);
        } else if (speed > 0 && (m_isRightBottom.get())) {
            m_rightWinch.set(0.0);
        } else {
            m_rightWinch.set(speed);
        }

        // left
        if (speed < 0.0 && (m_isLeftTop.get())) {
            m_leftWinch.set(0.0);
        } else if (speed > 0 && (m_isLeftBottom.get())) {
            m_leftWinch.set(0.0);
        } else {
            m_leftWinch.set(speed);
        }

    }

}