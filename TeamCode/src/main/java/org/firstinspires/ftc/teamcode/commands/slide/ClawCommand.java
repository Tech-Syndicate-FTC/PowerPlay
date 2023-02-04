package org.firstinspires.ftc.teamcode.commands.slide;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.slide.SlideSubsystem;

public class ClawCommand extends CommandBase {
    private final SlideSubsystem.ClawState state;

    private final SlideSubsystem subsystem;

    private ElapsedTime timer;

    public ClawCommand(SlideSubsystem subsystem, SlideSubsystem.ClawState clawState) {
        this.subsystem = subsystem;
        this.state = clawState;
    }

    @Override
    public void initialize() {
        subsystem.update(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}