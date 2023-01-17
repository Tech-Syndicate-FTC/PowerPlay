package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;

public class StrafeCommand extends CommandBase {
    public enum Directions {
        LEFT,
        RIGHT,
    }

    private DriveTrain drive;
    private double targetDistance;
    private Directions direction = Directions.RIGHT;

    public StrafeCommand(DriveTrain driveTrain, double target) {
        drive = driveTrain;
        targetDistance = target;
    }

    public StrafeCommand(DriveTrain driveTrain, double target, Directions direction) {
        drive = driveTrain;
        if (direction == Directions.RIGHT) {
            target = -target;
        }
        targetDistance = target;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).strafeLeft(targetDistance).build());
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}
