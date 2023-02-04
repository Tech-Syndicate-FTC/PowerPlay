package org.firstinspires.ftc.teamcode.commands.slide.auto_scoring;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.slide.ArmPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ClawCommand;
import org.firstinspires.ftc.teamcode.subsystems.slide.SlideSubsystem;

public class TransitionCommand extends SequentialCommandGroup {
    public TransitionCommand(SlideSubsystem subsystem) {
        super(
                new ClawCommand(subsystem, SlideSubsystem.ClawState.CLOSED),
                new ParallelCommandGroup(new ArmPositionCommand(subsystem, SlideSubsystem.ArmState.TRANSITION, 500))
        );
    }
}
