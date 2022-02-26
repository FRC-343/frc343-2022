package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableRegistry;


public class Shooter extends SubsystemBase {
    // private static final double kShootGarbage = 150.0; // rev per sec, for irregular values

    private final CANSparkMax m_bottomShooter = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax m_topShooter = new CANSparkMax(2, MotorType.kBrushless);

    private final RelativeEncoder m_bottomShooterEncoder = m_bottomShooter.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    private final RelativeEncoder m_topShooterEncoder = m_topShooter.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    private final PIDController m_shooterPIDController = new PIDController(0.10, 0.0, 0.0);
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(1.71, 0.0782);

    private double m_bottomSpeed = 0.0;
    private double m_topSpeed = 0.0;

    public Shooter() {
        m_bottomShooter.setInverted(false);
        m_bottomShooterEncoder.setVelocityConversionFactor(0.01666); //vel deffaults to RPM, this turns it to Rev/sec

        m_topShooter.setInverted(true);
        m_topShooterEncoder.setVelocityConversionFactor(0.01666);

        SendableRegistry.setSubsystem(m_shooterPIDController, this.getClass().getSimpleName());
        SendableRegistry.setName(m_shooterPIDController, "Shooter PIDController");
    }

    public double getBottomShooterRPS() {
        return m_bottomShooterEncoder.getVelocity();
    }

    public double getTopShooterRPS() {
        return m_topShooterEncoder.getVelocity();
    }

    public void shoot(double bottomSpeed, double topSpeed) {
        m_bottomSpeed = bottomSpeed;
        m_topSpeed = topSpeed;
    }

    public void shoot (double bothShooterSpeed) {
        shoot(bothShooterSpeed, bothShooterSpeed);
    }

    @Override
    public void periodic() {
        if (m_bottomSpeed > 0.01) {
            double shooterFeedforward = m_shooterFeedforward.calculate(m_bottomSpeed);
            double shooterPIDOutput = m_shooterPIDController.calculate(getBottomShooterRPS(), m_bottomSpeed);
            double shooterOutput = shooterFeedforward + shooterPIDOutput;

            m_bottomShooter.setVoltage(shooterOutput);
        } else {
            m_bottomShooter.setVoltage(0.0);
        }

        if (m_topSpeed > 0.01) {
            double shooterFeedforward = m_shooterFeedforward.calculate(m_topSpeed);
            double shooterPIDOutput = m_shooterPIDController.calculate(getTopShooterRPS(), m_topSpeed);
            double shooterOutput = shooterFeedforward + shooterPIDOutput;

            m_topShooter.setVoltage(shooterOutput);
        } else {
            m_topShooter.setVoltage(0.0);
        }
    }

    public void set(double topSpeed, double bottomSpeed) {
        m_topShooter.set(topSpeed);
        m_bottomShooter.set(bottomSpeed);
        System.out.println("top speed = " + getTopShooterRPS());
        System.out.println("bottom speed = " + getBottomShooterRPS());
        System.out.println("done");
    }
}
