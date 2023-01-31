package org.firstinspires.ftc.teamcode.commands.slide;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.slide.SlideSubsystem;

public class SlidePositionCommand extends CommandBase {
    private final double position;
    private final double timeout;
    private final double max_v;
    private final double max_a;
    private final double allowed_error;

    private final SlideSubsystem subsystem;
    private final SlideSubsystem.STATE errorState;

    private ElapsedTime timer;

    public SlidePositionCommand(SlideSubsystem intake, double position, double v, double a,
                                double allowed_error, double timeout, SlideSubsystem.STATE error) {
        this.position = position;
        this.timeout = timeout;
        this.subsystem = intake;
        this.max_v = v;
        this.max_a = a;
        this.allowed_error = allowed_error;
        this.errorState = error;
    }

    @Override
    public void execute() {
        if (timer == null) {
            timer = new ElapsedTime();
            subsystem.newProfile(position, max_v, max_a);
        }

        if (timer.milliseconds() > timeout) {
            subsystem.state = errorState;
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(subsystem.getPos() - position) < allowed_error || timer.milliseconds() > timeout;
    }
}