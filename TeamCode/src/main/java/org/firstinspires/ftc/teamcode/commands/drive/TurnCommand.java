package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;

public class TurnCommand extends CommandBase {
    private DriveTrain drive;
    private double targetDegree;

    public TurnCommand(DriveTrain driveTrain, double target) {
        drive = driveTrain;
        targetDegree = target;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).turn(Math.toRadians(targetDegree)).build());
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}
