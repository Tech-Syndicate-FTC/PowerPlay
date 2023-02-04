package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SharedStates;
import org.firstinspires.ftc.teamcode.commands.slide.ClawCommand;
import org.firstinspires.ftc.teamcode.helpers.gamepad.PS4Buttons;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.slide.SlideManager;
import org.firstinspires.ftc.teamcode.subsystems.slide.SlideSubsystem;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends BaseOpMode {
    private double loopTime = 0;
    public SlideSubsystem slideSubsystem;
    public SlideManager slideManager;

    @Override
    public void onInit() {
        slideSubsystem = new SlideSubsystem(hardwareMap, false);
        slideSubsystem.update(SlideSubsystem.ArmState.TRANSITION);
        slideManager = new SlideManager(slideSubsystem);

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
        pilot.getGamepadButton(PS4Buttons.SQUARE).whenPressed(() -> {
            drive.drivePrecision = drive.drivePrecision.toggle();
        });

        pilot.getGamepadButton(PS4Buttons.CIRCLE).whenPressed(() -> {
            drive.drivePrecision = DriveTrain.DrivePrecision.MANUAL;
        });

        pilot.getGamepadButton(PS4Buttons.TRIANGLE).whenPressed(() -> {
            drive.driveType = drive.driveType.toggle();
        });

        // COPILOT Controls
        copilot.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            slideManager.reset();
        });

        copilot.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            slideManager.transition();
        });

        copilot.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            slideManager.score();
        });

        copilot.getGamepadButton(PS4Buttons.CROSS).whenPressed(() -> {
            slideManager.setup(SlideManager.STATE.LOW_BACK);
        });

        copilot.getGamepadButton(PS4Buttons.SQUARE).whenPressed(() -> {
            slideManager.setup(SlideManager.STATE.MEDIUM);
        });

        copilot.getGamepadButton(PS4Buttons.TRIANGLE).whenPressed(() -> {
            slideManager.setup(SlideManager.STATE.HIGH);
        });

        copilot.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ClawCommand(slideSubsystem, SlideSubsystem.ClawState.OPEN));
        copilot.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ClawCommand(slideSubsystem, SlideSubsystem.ClawState.CLOSED));
    }

    @Override
    public void onLoop() {
        drive.gamepadInput = new Vector2d(pilot.getLeftY(), -pilot.getLeftX());
        drive.gamepadInputTurn = pilot.getRightX();
        drive.manualPrecision = gamepad1.right_trigger;
        //slideSubsystem.setSlideFactor(copilot.getLeftY());

        drive.gamepadDrive();
        slideSubsystem.periodic();

        t.addData("pose", "(%3.2f, %3.2f, %3.2f°)", drive.getPoseX(), drive.getPoseY(), drive.getPoseHeading());
        t.addData("drive info", "%s %s", drive.driveType, drive.drivePrecision);
        t.addData("slide state", slideSubsystem.state);
        t.addData("slide manager state", slideManager.state);
        t.addData("arm angle", slideSubsystem.armAngle);
        t.addData("slide reality left/right, target", "%d %d, %d", (int) slideSubsystem.leftPosition, (int) slideSubsystem.rightPosition, (int) slideSubsystem.targetPosition);
        double loop = System.nanoTime();
        t.addData("hz ", 1000000000 / (loop - loopTime));
        t.update();
        loopTime = loop;
    }
}
