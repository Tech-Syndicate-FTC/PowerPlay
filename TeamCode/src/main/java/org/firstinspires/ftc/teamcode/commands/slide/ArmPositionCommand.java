package org.firstinspires.ftc.teamcode.commands.slide;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.slide.SlideSubsystem;

public class ArmPositionCommand extends CommandBase {
    private final SlideSubsystem.ArmState state;
    private final double timeout;

    private final SlideSubsystem subsystem;

    private ElapsedTime timer;
    private boolean alreadyInPosition = false;

    public ArmPositionCommand(SlideSubsystem subsystem, SlideSubsystem.ArmState armState, double timeout) {
        this.subsystem = subsystem;
        this.state = armState;
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        subsystem.update(state);
        timer = new ElapsedTime();
        alreadyInPosition = Math.abs(subsystem.armAngle - subsystem.armLeft.getAngle()) < 5;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > timeout || alreadyInPosition;
    }
}