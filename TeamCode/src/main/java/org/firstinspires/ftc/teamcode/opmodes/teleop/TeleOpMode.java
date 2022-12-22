package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "Competition TeleOp", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public DriveTrain drive;
    private Gamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drive = new DriveTrain(hardwareMap);

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.clearAll();
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            /*
            drive.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
            gamepad = gamepad1;
            drive.gamepadInput = new Vector2d(-gamepad.left_stick_y, -gamepad.left_stick_x);
            drive.gamepadInputTurn = -gamepad.right_stick_x;

            if (gamepad.right_trigger > 0.25) {
                drive.arm.closeClawManual();
            }

            if (gamepad.left_trigger > 0.25) {
                drive.arm.openClawManual();
            }

            if (gamepad.y) {
                drive.arm.slideUp();
                drive.elevator.slide(Elevator.Direction.UP,1);
            } else if (gamepad.a) {
                drive.elevator.slide(Elevator.Direction.DOWN,1);
            }

            drive.driveTrainPointFieldModes();
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", String.format(Locale.ENGLISH, "%3.2f", poseEstimate.getX()));
            telemetry.addData("y", String.format(Locale.ENGLISH, "%3.2f", poseEstimate.getY()));
            telemetry.addData("heading", String.format(Locale.ENGLISH, "%3.2f°", poseEstimate.getHeading()));
            telemetry.addData("claw", String.format(Locale.ENGLISH, "%3.0f", drive.arm.getClawPosition()));
            telemetry.addData("slide", String.format(Locale.ENGLISH, "%1.0f", drive.arm.getSlidePower()));
            telemetry.addData("slide_pos", drive.arm.getSlidePosition());
            telemetry.update();
            */
        }
    }
}
