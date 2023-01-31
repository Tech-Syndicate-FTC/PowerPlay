package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SharedStates;
import org.firstinspires.ftc.teamcode.helpers.gamepad.PS4Keys;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends BaseOpMode {
    @Override
    public void onInit() {
        if (SharedStates.currentPose != null) {
            drive.setExternalHeading(SharedStates.currentPose.getHeading());
            SharedStates.currentPose = null;
        } else {
            drive.setExternalHeading(0);
        }
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void onStart() {
        // PILOT Controls
        pilot.getGamepadButton(PS4Keys.SQUARE).whenPressed(() -> {
            drive.driveMode = drive.driveMode.toggle();
        });

        pilot.getGamepadButton(PS4Keys.TRIANGLE).whenPressed(() -> {
            drive.driveMode = drive.driveMode.toggle();
        });
    }

    @Override
    public void onLoop() {
        drive.gamepadInput = new Vector2d(pilot.getLeftY(), -pilot.getLeftX());
        drive.gamepadInputTurn = -pilot.getRightX();

        drive.gamepadDrive();

        Pose2d poseEstimate = drive.getPoseEstimate();
        t.addData("pose", "(%3.2f, %3.2f, %3.2f°)", poseEstimate.getX(), poseEstimate.getY(), Math.toDegrees(drive.getExternalHeading()));
        t.addData("drive info:", "%s %s", drive.driveType, drive.driveMode);
        t.update();
    }
}
