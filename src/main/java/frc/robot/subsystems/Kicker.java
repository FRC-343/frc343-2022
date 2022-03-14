package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {
    private final Spark m_kicker = new Spark(4);

    private final ColorSensorV3 m_color = new ColorSensorV3(I2C.Port.kOnboard);

    private final ColorMatch m_colorMatcher = new ColorMatch();

    private static final Color kRed = new Color(0.518311, 0.344971, 0.136963);
    private static final Color kBlue = new Color(0.1267, 0.4160, 0.4575);

    private String colorString = "";

    public Kicker() {
        m_kicker.setInverted(true);

        SendableRegistry.setSubsystem(m_kicker, this.getClass().getSimpleName());
        SendableRegistry.setName(m_kicker, "Kicker Motor");

        m_colorMatcher.addColorMatch(kRed);
        m_colorMatcher.addColorMatch(kBlue);
    }

    @Override
    public void periodic() {
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

        String s = m_color.getRed() + ", " + m_color.getGreen() + ", " + m_color.getBlue();
        SmartDashboard.putString("raw color", s);

    }

    public void setKicker(double speed) {
        m_kicker.set(speed);
    }

    public boolean isBadCargo() { // returns true if wrong color
        if (colorString.isBlank()) {
            return false;
        } else {
            return !(DriverStation.getAlliance().equals(DriverStation.Alliance.valueOf(colorString)));
        }
    }
}
