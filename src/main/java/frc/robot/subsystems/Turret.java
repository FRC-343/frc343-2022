package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private final Spark m_turret = new Spark(5);
    private final Encoder m_turretEncoder = new Encoder(0, 1);
    private final DigitalInput m_isLeft = new DigitalInput(6); // has completely turned counter clockwise
    private final DigitalInput m_isRight = new DigitalInput(7); // has completely turned clockwise

    private double m_speed = 0.0;
    private boolean m_aiming = false; // aiming is just used for getting to a preset angle
    private boolean m_aimed = false; // same
    private boolean m_zeroing = false; // reseting to get good encoder values
    private double m_target = 0.0;

    public Turret() {

        m_turret.setInverted(true);
        m_turretEncoder.setReverseDirection(true);

        SendableRegistry.setSubsystem(m_turret, this.getClass().getSimpleName());
        SendableRegistry.setName(m_turret, "turret motor");

        SendableRegistry.setSubsystem(m_turretEncoder, this.getClass().getSimpleName());
        SendableRegistry.setName(m_turretEncoder, "Turret Encoder");

        SendableRegistry.setSubsystem(m_isLeft, this.getClass().getSimpleName());
        SendableRegistry.setName(m_isLeft, "Turret left (ccw) limit switch");

        SendableRegistry.setSubsystem(m_isRight, this.getClass().getSimpleName());
        SendableRegistry.setName(m_isRight, "Turret right (cw) limit switich");
    }

    public void stop() {
        m_turret.set(0.0);
    }

    public boolean isAimed() {
        return m_aimed;
    }

    public void spin(double speed) {
        m_aiming = false;
        m_aimed = false;
        m_speed = speed;
    }

    public void aim(double target) {
        m_target = target;

        if (!m_aiming) {
            m_aiming = true;
            m_zeroing = true;
        }
    }

    private void rotate(double maxSpeed) { // use this method instead of the .set() to use dynamic speed basic on how close it is to the limit switch
        boolean isPositiveSpeed = maxSpeed > 0;

        if (!isPositiveSpeed) { // if ccw
            if (m_turretEncoder.get() < 15) {
                m_turret.set(-.2);
            } else if (m_turretEncoder.get() < 45) {
                m_turret.set(-.5);
            } else { // not close to left limit switch
                m_turret.set(-1.0);
            }
        } else if (isPositiveSpeed) {
            if (m_turretEncoder.get() > 230) {
                m_turret.set(.2);
            } else if (m_turretEncoder.get() > 200) {
                m_turret.set(.5);
            } else { // not close to right limit switch
                m_turret.set(1.0);
            }
        }
    }

    @Override
    public void periodic() {
        if (m_aiming) { // trying to get to a certain value

            if (m_isLeft.get()) { // check if already zeroed (gone all the way left)
                m_zeroing = false;
                m_turretEncoder.reset();
            }

            if (m_zeroing) { // reseting for proper measurement
                // m_turret.set(-.5);
                rotate(-1);
            } else {
                if (m_isRight.get()) { // went all the way to the right, so try again
                    m_zeroing = true;
                    stop();
                } else if (m_turretEncoder.getDistance() < m_target) { // going towards target cw
                    // m_turret.set(.5);
                    rotate(1);
                } else {
                    stop();
                    m_aimed = true;
                }
            }

            SmartDashboard.putBoolean("turret reached preset", m_aimed);

        } else {
            // limit switch logic assuming possible motor values = cw, and negative = ccw
            if (m_isLeft.get() && m_speed < 0.0) {
                stop();
                m_turretEncoder.reset();
            } else if (m_isRight.get() && m_speed > 0.0) {
                stop();
            } else {
                rotate(m_speed); // not touching limit switches or touching a limit switch but tring to go the
                                 // oppisite way
            }
        }

    }

}
