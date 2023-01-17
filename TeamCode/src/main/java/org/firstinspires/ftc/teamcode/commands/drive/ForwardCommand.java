package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;

public class ForwardCommand extends CommandBase {
    private DriveTrain drive;
    private double targetDistance;

    public ForwardCommand(DriveTrain driveTrain, double target) {
        drive = driveTrain;
        targetDistance = target;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).forward(targetDistance).build());
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}
