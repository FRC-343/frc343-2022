package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.ShootingRelatingCommands.ShootCommand;

public class Kicker extends SubsystemBase {
    private static final Kicker m_instance = new Kicker();
    public static double activateShooter[] = { 0, 0 }; // bottom speed, top speed

    private final Spark m_kicker = new Spark(4);

    private final DigitalInput m_cellDetector = new DigitalInput(14);

    private final ColorSensorV3 m_color = new ColorSensorV3(I2C.Port.kOnboard);

    private final ColorMatch m_colorMatcher = new ColorMatch();

    private static final Color kRed = new Color(0.518311, 0.344971, 0.136963);
    private static final Color kBlue = new Color(0.1267, 0.4160, 0.4575);

    private String colorString = "";

    public Kicker() {

        m_kicker.setInverted(true);

        SendableRegistry.setSubsystem(m_kicker, this.getClass().getSimpleName());
        SendableRegistry.setName(m_kicker, "Kicker Motor");

        SendableRegistry.setSubsystem(m_cellDetector, this.getClass().getSimpleName());
        SendableRegistry.setName(m_cellDetector, "cell detector for shooter/intake");

        m_colorMatcher.addColorMatch(kRed);
        m_colorMatcher.addColorMatch(kBlue);
    }

    public static Kicker getInstance() {
        return m_instance;
    }

    @Override
    public void periodic() {

        // running kicker motor

        if (ShootCommand.activateKicker != 0) {
            setKicker(ShootCommand.activateKicker);
        } else if (runKickerForIntake()) {
            kickerForIntake();
        } else {
            setKicker(0);
        }

        // color sensor things
        ColorMatchResult detectedColor = m_colorMatcher.matchClosestColor(m_color.getColor());
        if (detectedColor.color == kRed) {
            SmartDashboard.putString("color_detected", "red");
            colorString = "Red";
        } else if (detectedColor.color == kBlue) {
            SmartDashboard.putString("color_detected", "blue");
            colorString = "Blue";
        } else {
            SmartDashboard.putString("color_detected", "None Colors there be");
            colorString = "";
        }

    }

    public void setKicker(double speed) {
        m_kicker.set(speed);
    }

    public boolean isBadCargo() { // returns true if wrong color
        boolean value;
        if (colorString.isBlank()) {
            value = false;
        } else if (!getCellDetector()) {
            value = false;
        } else {
            value = !(DriverStation.getAlliance().equals(DriverStation.Alliance.valueOf(colorString)));
        }
        return value;
    }

    public boolean getCellDetector() {
        return m_cellDetector.get();
    }

    private void shoot(double bottom, double top) {
        activateShooter[0] = bottom;
        activateShooter[1] = top;

    }

    private void kickerForIntake() {
        if (!getCellDetector()) { // if no ball is in chamber run the kicker so it goes into chanber // room for the 2nd ball in the hopper
            shoot(0, 0);
            setKicker(1.0);
        } else { // if getCellDetector()
            if (Robot.kUseColorSensorIntake) {
                if (!isBadCargo()) { // if good cargo stop kicker
                    setKicker(0.0);
                    shoot(0, 0);
                } else if (isBadCargo()) { // if bad then shoot out
                    shoot(50, -95);
                    setKicker(1.0);
                }
            } else { // getCellDectector: ball in chamber
                setKicker(0.0);
                shoot(0, 0);
            }
        }

    }

    private boolean runKickerForIntake() {
        return Intake.isRunning();
    }
}
