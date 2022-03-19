package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climbing;

public class AutoClimbCommand extends SequentialCommandGroup {

    public AutoClimbCommand(Climbing climbing) {

        addCommands(
                // raise arms manually before and move robot to correct position

                // bring in hooks
                new InstantCommand(climbing::disEngage, climbing),
                // lower arms
                new ClimbArmCommand(climbing, .8), // positive should be down, auto stops when reaches bottom/top depending on direction
                // bring out hooks to grab mid rung
                new InstantCommand(climbing::engage, climbing),
                // wait b/c they are slow
                new WaitCommand(1),
                new ParallelDeadlineGroup(
                        new WaitCommand(1,
                        new ClimbArmCommand(climbing, -.8)),
                // bring them back in so climbing arms can extend out again
                new InstantCommand(climbing::disEngage, climbing),
                new ClimbArmCommand(climbing, -.8),
                new InstantCommand(climbing::engage, climbing),
                new WaitCommand(1),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.3),
                new ClimbArmCommand(climbing, .8)));
    }
}
