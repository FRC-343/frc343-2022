package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final DoubleSolenoid m_intakeLift = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 1, 0);
    private final Spark m_intake = new Spark(6);
    private final DigitalInput m_cellDetector = new DigitalInput(14);

    public Intake() {
        m_intake.setInverted(true);

        SendableRegistry.setSubsystem(m_intake, this.getClass().getSimpleName());
        SendableRegistry.setName(m_intake, "Intake Motor");

        SendableRegistry.setSubsystem(m_intakeLift, this.getClass().getSimpleName());
        SendableRegistry.setName(m_intakeLift, "Intake Lift");

        SendableRegistry.setSubsystem(m_cellDetector, this.getClass().getSimpleName());
        SendableRegistry.setName(m_cellDetector, "cell detector for shooter/intake");
    }

    public void raise() {
        m_intakeLift.set(DoubleSolenoid.Value.kReverse);
    }

    public void lower() {
        m_intakeLift.set(DoubleSolenoid.Value.kForward);
    }

    public void setIntake(double speed) {
        m_intake.set(speed);
    }

    public boolean getCellDetector() {
        return m_cellDetector.get();
    }
}
