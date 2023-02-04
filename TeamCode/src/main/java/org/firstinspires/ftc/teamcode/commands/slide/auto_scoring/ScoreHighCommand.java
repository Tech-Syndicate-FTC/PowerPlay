package org.firstinspires.ftc.teamcode.commands.slide.auto_scoring;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.slide.ArmPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.slide.SlideSubsystem;

public class ScoreHighCommand extends SequentialCommandGroup {
    public ScoreHighCommand(SlideSubsystem subsystem) {
        super(
                new ArmPositionCommand(subsystem, SlideSubsystem.ArmState.SCORE, 500),
                new ClawCommand(subsystem, SlideSubsystem.ClawState.OPEN),
                new ArmPositionCommand(subsystem, SlideSubsystem.ArmState.TRANSITION, 700),
                new LiftPositionCommand(subsystem, 0, 20, 1500, SlideSubsystem.STATE.FAILED_RETRACT),
                new ClawCommand(subsystem, SlideSubsystem.ClawState.OPEN)
        );
    }
}
