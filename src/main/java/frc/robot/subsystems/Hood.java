package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    private final Encoder m_hoodEncoder = new Encoder(4, 5);
    private final DigitalInput m_hoodBack = new DigitalInput(2); // 2 = bottom = back
    private final DigitalInput m_hoodFront = new DigitalInput(3);
    private final Spark m_hoodMotor = new Spark(7);

    private final double kMaxHoodEncoderValue = 4000; // TODO change here also
    private final double kMaxHoodEncoderRate = 1700;

    private boolean m_aimed = false; // if shooter is currently aimed
    private double m_target = 0.0; // where it needs to be aiming
    private double m_speed = 0.0; // manual control
    private boolean m_aiming = false; // if currently aiming (for automatic)
    private boolean m_zeroing = false; // resetting hood

    public Hood() {
        m_hoodMotor.setInverted(false); // I think either the hood should be inverted or the encoder should be but not both
        m_hoodEncoder.setReverseDirection(false); // it failed when both false and both true, so try true-false and false-true

        SendableRegistry.setSubsystem(m_hoodEncoder, this.getClass().getSimpleName());
        SendableRegistry.setName(m_hoodEncoder, "Hood Encoder");
        SendableRegistry.setSubsystem(m_hoodBack, this.getClass().getSimpleName());
        SendableRegistry.setName(m_hoodBack, "Hood Back Limit");
        SendableRegistry.setSubsystem(m_hoodFront, this.getClass().getSimpleName());
        SendableRegistry.setName(m_hoodFront, "Hood Front Limit");
        SendableRegistry.setSubsystem(m_hoodMotor, this.getClass().getSimpleName());
        SendableRegistry.setName(m_hoodMotor, "Hood Motor");

    }

    public void aim(double target) {
        m_target = target;
        SmartDashboard.putNumber("hood_target", m_target);

        if (!m_aiming) {
            m_aiming = true;
            // m_zeroing = true;
        }
    }

    public void stop() {
        m_hoodMotor.set(0.0);
    }

    public boolean isAimed() {
        return m_aimed;
    }

    public void move(double speed) {
        m_aiming = false;
        m_aimed = false;
        m_speed = speed;
    }

    @Override
    public void periodic() {
        if (m_aiming) {

            if (m_hoodEncoder.getRate() > kMaxHoodEncoderRate || m_hoodEncoder.getRate() < -kMaxHoodEncoderRate
                    || m_hoodEncoder.getDistance() > kMaxHoodEncoderValue
                    || m_hoodEncoder.getDistance() < -200) {
                System.err.println("Hood encoder sent garbage values, zeroing again...");
                m_zeroing = true;
            }

            if (m_hoodBack.get()) {
                m_zeroing = false;
                m_hoodEncoder.reset();
            }


            if (m_zeroing) {
                m_hoodMotor.set(1.0);
            } else {
                if (m_hoodFront.get()) {
                    m_zeroing = true;
                    m_hoodMotor.set(0.0);
                } else if (m_hoodEncoder.getDistance() < m_target - 100) {
                    m_hoodMotor.set(-1.0);
                } else if (m_hoodEncoder.getDistance() > m_target + 200) {
                    m_hoodMotor.set(1.0);
                } else { //m_hoodEncoder.getDistance >m_target-100 && < m_target+200
                    m_hoodMotor.set(0.0);
                    m_aimed = true;
                }
            }
            SmartDashboard.putBoolean("hood aimed", m_aimed);

        } else {
            if (m_hoodBack.get() && m_speed > 0.0) {
                m_hoodMotor.set(0.0);
                m_hoodEncoder.reset();
            } else if (m_hoodFront.get() && m_speed < 0.0) {
                m_hoodMotor.set(0.0);
            } else {
                m_hoodMotor.set(m_speed);
            }

        }
    }
}
