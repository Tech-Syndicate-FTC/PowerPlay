package org.firstinspires.ftc.teamcode.commands.slide;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.slide.SlideSubsystem;

public class LiftPositionCommand extends CommandBase {
    private final double position;
    private double timeout = 0;
    private final double max_v;
    private final double max_a;
    private final double allowed_error;

    private final SlideSubsystem subsystem;
    private SlideSubsystem.STATE errorState = null;

    private ElapsedTime timer;

    public LiftPositionCommand(SlideSubsystem subsystem, double position, double allowed_error) {
        this.position = position;
        this.subsystem = subsystem;
        this.max_v = 6000;
        this.max_a = 5000;
        this.allowed_error = allowed_error;
    }

    public LiftPositionCommand(SlideSubsystem subsystem, double position, double allowed_error, double timeout, SlideSubsystem.STATE error) {
        this.position = position;
        this.timeout = timeout;
        this.subsystem = subsystem;
        this.max_v = 6000;
        this.max_a = 5000;
        this.allowed_error = allowed_error;
        this.errorState = error;
    }

    public LiftPositionCommand(SlideSubsystem subsystem, double position, double v, double a,
                               double allowed_error, double timeout, SlideSubsystem.STATE error) {
        this.position = position;
        this.timeout = timeout;
        this.subsystem = subsystem;
        this.max_v = v;
        this.max_a = a;
        this.allowed_error = allowed_error;
        this.errorState = error;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        subsystem.newProfile(position, max_v, max_a);
    }

    @Override
    public void execute() {
        if (timeout != 0 && timer.milliseconds() > timeout) {
            subsystem.state = errorState;
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(subsystem.getPos() - position) < allowed_error || (timeout != 0 && timer.milliseconds() > timeout);
    }
}