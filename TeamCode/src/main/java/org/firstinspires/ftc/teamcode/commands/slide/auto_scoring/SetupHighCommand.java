package org.firstinspires.ftc.teamcode.commands.slide.auto_scoring;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.slide.ArmPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.slide.SlideSubsystem;

public class SetupHighCommand extends ParallelCommandGroup {
    public SetupHighCommand(SlideSubsystem subsystem) {
        super(
                new LiftPositionCommand(subsystem, 2200, 20, 2000, SlideSubsystem.STATE.FAILED_EXTEND),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new ArmPositionCommand(subsystem, SlideSubsystem.ArmState.AIM, 1000)

                )
        );
    }
}
