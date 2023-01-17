package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;

public class PositionCommand extends CommandBase {
    private DriveTrain drive;
    private Pose2d targetPose;

    public PositionCommand(DriveTrain driveTrain, Pose2d target) {
        drive = driveTrain;
        targetPose = new Pose2d(target.getY(), target.getX(), Math.toRadians(360 - Math.toDegrees(target.getHeading())));
    }

    public PositionCommand(DriveTrain driveTrain, Vector2d target) {
        drive = driveTrain;
        targetPose = new Pose2d(target.getY(), target.getX(), 0);
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToLinearHeading(targetPose).build());
    }
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            //drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}
