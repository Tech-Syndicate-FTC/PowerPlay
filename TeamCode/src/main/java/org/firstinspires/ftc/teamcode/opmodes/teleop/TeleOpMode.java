package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions.Ground;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions.High;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions.Low;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions.Medium;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.IDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystems.SharedStates;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends BaseOpMode {

    @Override
    public void onInit() {
        elevator.setState(SharedStates.elevatorState);
        if (SharedStates.currentPose != null) {
            drive.setExternalHeading(SharedStates.currentPose.getHeading());
            SharedStates.currentPose = null;
        } else {
            drive.setExternalHeading(0);
        }
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void onInitLoop() {
        elevator.runStateMachine();
        elevator.update();
    }

    @Override
    public void onStart() {
        elevator.setLiftTargetPosition(Elevator.ELEVATOR_HOME);

        copilot.Buttons.Triangle.onPress(() -> {
            elevator.setLevel(High);
        });

        copilot.Buttons.Circle.onPress(() -> {
            elevator.setLevel(Medium);
        });

        copilot.Buttons.Square.onPress(() -> {
            elevator.setLevel(Low);
        });

        copilot.Buttons.X.onPress(() -> {
            elevator.setLevel(Ground);
        });

        copilot.Triggers.Right.onPress(() -> {
            elevator.closeClaw();
        });

        copilot.Triggers.Left.onPress(() -> {
            elevator.openClaw();
        });
    }

    @Override
    public void onLoop() {
        drive.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        drive.gamepadInput = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        drive.gamepadInputTurn = -gamepad1.right_stick_x;

        drive.driveTrainPointFieldModes();

        if (copilot.Buttons.Share.isPressed() && copilot.Buttons.Options.isPressed()) {
            elevator.setState(IDLE);
        }

        t.addData("Elevator", elevator.getStateText());
        elevator.showElevatorState();

        Pose2d poseEstimate = drive.getPoseEstimate();
        t.addData("x", String.format("%3.2f", poseEstimate.getX()));
        t.addData("y", String.format("%3.2f", poseEstimate.getY()));
        t.addData("heading", String.format("%3.2f", Math.toDegrees(drive.getExternalHeading())));
        t.update();
    }
}
