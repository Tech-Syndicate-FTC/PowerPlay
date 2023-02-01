package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SharedStates;
import org.firstinspires.ftc.teamcode.helpers.gamepad.PS4Keys;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.slide.SlideSubsystem;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends BaseOpMode {
    private double loopTime = 0;
    public SlideSubsystem slideSubsystem;

    @Override
    public void onInit() {
        slideSubsystem = new SlideSubsystem(hardwareMap, false);
        slideSubsystem.update(SlideSubsystem.ArmState.TRANSITION);

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
            drive.drivePrecision = drive.drivePrecision.toggle();
        });

        pilot.getGamepadButton(PS4Keys.CIRCLE).whenPressed(() -> {
            drive.drivePrecision = DriveTrain.DrivePrecision.MANUAL;
        });

        pilot.getGamepadButton(PS4Keys.TRIANGLE).whenPressed(() -> {
            drive.driveType = drive.driveType.toggle();
        });

        // COPILOT Controls
        copilot.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            slideSubsystem.update(SlideSubsystem.ArmState.INTAKE);
        });

        copilot.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            slideSubsystem.update(SlideSubsystem.ArmState.TRANSITION);
        });

        copilot.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            slideSubsystem.update(SlideSubsystem.ArmState.HOME);
        });

        copilot.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            slideSubsystem.update(SlideSubsystem.ArmState.UPRIGHT);
        });

        copilot.getGamepadButton(PS4Keys.CROSS).whenPressed(() -> {
            slideSubsystem.update(SlideSubsystem.ArmState.SCORE_LOW);
        });
    }

    @Override
    public void onLoop() {
        drive.gamepadInput = new Vector2d(pilot.getLeftY(), -pilot.getLeftX());
        drive.gamepadInputTurn = pilot.getRightX();
        drive.manualPrecision = gamepad1.right_trigger;

        drive.gamepadDrive();

        t.addData("pose", "(%3.2f, %3.2f, %3.2f°)", drive.getPoseX(), drive.getPoseY(), drive.getPoseHeading());
        t.addData("drive info", "%s %s", drive.driveType, drive.drivePrecision);
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        t.update();
        slideSubsystem.periodic();
    }
}
