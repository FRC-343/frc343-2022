package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AimShootCommand extends SequentialCommandGroup {

    public AimShootCommand(Vision vision, Hood hood, Turret turret, Shooter shooter, Kicker kicker,
            boolean stopForTime) {
        addCommands(new ParallelCommandGroup(
                new AimCommand(vision, hood, turret),
                new ShootCommand(shooter, kicker, true, stopForTime)));
    }

}
