package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorState;
import org.firstinspires.ftc.teamcode.subsystems.HubPerformance;
import org.firstinspires.ftc.teamcode.subsystems.SharedStates;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;

import java.util.Locale;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "Test TeleOp", group = "Teleop")
public class TestTeleOp extends LinearOpMode {
    public DriveTrain drive;
    private Gamepad gamepad;
    private Elevator elevator;

    boolean lastStepUp = false;
    boolean lastStepDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        HubPerformance.enable(hardwareMap);

        drive = new DriveTrain(hardwareMap);
        elevator = new Elevator(this, false);

        elevator.setState(SharedStates.elevatorState);
        if (SharedStates.currentPose != null) {
            drive.setExternalHeading(SharedStates.currentPose.getHeading());
            SharedStates.currentPose = null;
        } else {
            drive.setExternalHeading(0);
        }

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.clearAll();
        telemetry.update();

        //waitForStart();

        while (opModeInInit()) {
            elevator.runStateMachine();
            telemetry.addData("GYRO heading", Math.toDegrees(drive.getExternalHeading()));
            telemetry.update();
        }

        elevator.setLiftTargetPosition(Elevator.ELEVATOR_HOME);

        while (!isStopRequested()) {
            drive.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
            gamepad = gamepad1;
            drive.gamepadInput = new Vector2d(-gamepad.left_stick_y, -gamepad.left_stick_x);
            drive.gamepadInputTurn = -gamepad.right_stick_x;

            /*if (gamepad.right_trigger > 0.25) {
                //drive.arm.closeClawManual();
            }

            if (gamepad.left_trigger > 0.25) {
                //drive.arm.openClawManual();
            }

            if (gamepad.y) {
                //drive.arm.slideUp();
            } else if (gamepad.a) {
            }*/

            boolean stepUp = gamepad.dpad_up;
            boolean stepDown = gamepad.dpad_down;
            boolean newPosition = false;

            // reset Home on Elevator
            if (gamepad.back && gamepad.start) {
                elevator.setState(ElevatorState.IDLE);
            }

            //elevator.driverManualGrabRequest = gamepad.square;

            if (stepUp && !lastStepUp) {
                elevator.levelUp();
            }
            if (stepDown && !lastStepDown) {
                elevator.levelDown();
            }

            lastStepUp   = stepUp;
            lastStepDown = stepDown;

            // Manually jog the elevator.
            //elevator.jogElevator(-gamepad2.left_stick_y);

            // Display Telemetry data
            //telemetry.addData("GYRO heading", Math.toDegrees(drive.getExternalHeading()));
            //telemetry.update();

            drive.driveTrainPointFieldModes();
            drive.update();
            elevator.update();
            elevator.runStateMachine();

            telemetry.addData("Elevator", elevator.getStateText());
            elevator.showElevatorState();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", String.format(Locale.ENGLISH, "%3.2f", poseEstimate.getX()));
            telemetry.addData("y", String.format(Locale.ENGLISH, "%3.2f", poseEstimate.getY()));
            telemetry.addData("heading", String.format(Locale.ENGLISH, "%3.2f°", poseEstimate.getHeading()));
            //telemetry.addData("claw", String.format(Locale.ENGLISH, "%3.0f", drive.arm.getClawPosition()));
            //telemetry.addData("slide", String.format(Locale.ENGLISH, "%1.0f", drive.arm.getSlidePower()));
            //telemetry.addData("slide_pos", drive.arm.getSlidePosition());
            telemetry.update();
        }
    }
}
