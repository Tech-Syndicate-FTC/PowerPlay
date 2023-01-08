package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.HubPerformance;
import org.firstinspires.ftc.teamcode.subsystems.SharedStates;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState;

import java.util.Locale;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "TeleOp v2", group = "Teleop")
public class NewTeleOp extends LinearOpMode {
    public DriveTrain drive;
    private Gamepad pilot;
    private Gamepad copilot;
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

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Init

        while (opModeInInit()) {
            elevator.runStateMachine();
            telemetry.addData("GYRO heading", Math.toDegrees(drive.getExternalHeading()));
            telemetry.update();
        }

        elevator.setLiftTargetPosition(Elevator.ELEVATOR_HOME);

        // Loop

        while (!isStopRequested()) {
            drive.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
            pilot = gamepad1;
            copilot = gamepad2;

            drive.gamepadInput = new Vector2d(-pilot.left_stick_y, -pilot.left_stick_x);
            drive.gamepadInputTurn = -pilot.right_stick_x;

            // Co-pilot controls
            boolean stepUp = copilot.dpad_up;
            boolean stepDown = copilot.dpad_down;
            boolean newPosition = false;

            // reset Home on Elevator
            if (copilot.back && copilot.start) {
                elevator.setState(ElevatorState.IDLE);
            }

            //elevator.driverManualGrabRequest = gamepad.square;

            if (stepUp && !lastStepUp) {
                elevator.levelUp();
            }
            if (stepDown && !lastStepDown) {
                elevator.levelDown();
            }

            lastStepUp = stepUp;
            lastStepDown = stepDown;

            if (copilot.a) {
                elevator.setLevel(Elevator.Junctions.Ground);
            } else if (copilot.b) {
                elevator.setLevel(Elevator.Junctions.Low);
            } else if (copilot.x) {
                elevator.setLevel(Elevator.Junctions.Medium);
            } else if (copilot.y) {
                elevator.setLevel(Elevator.Junctions.High);
            }

            if (copilot.right_trigger > 0.5) {
                elevator.closeClaw();
            }

            if (copilot.left_trigger > 0.5) {
                elevator.openClaw();
            }

            // Manually jog the elevator.
            if (copilot.left_stick_y != 0) {
                elevator.jogElevator(-copilot.left_stick_y);
                elevator.newLevelRequested = true;
            }
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
            telemetry.addData("x", "%3.2f", poseEstimate.getX());
            telemetry.addData("y", "%3.2f", poseEstimate.getY());
            telemetry.addData("heading", "%3.2f°", poseEstimate.getHeading());
            telemetry.addData("GYRO heading", Math.toDegrees(drive.getExternalHeading()));
            //telemetry.addData("Slide Pos", elevator.getLiftRawPosition());
            //telemetry.addData("claw", String.format(Locale.ENGLISH, "%3.0f", drive.arm.getClawPosition()));
            //telemetry.addData("slide", String.format(Locale.ENGLISH, "%1.0f", drive.arm.getSlidePower()));
            //telemetry.addData("slide_pos", drive.arm.getSlidePosition());
            telemetry.update();
        }
    }
}
