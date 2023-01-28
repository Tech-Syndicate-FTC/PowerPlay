package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions.Ground;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions.High;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions.Low;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions.Medium;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions.Stack;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.SharedStates;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends BaseOpMode {
    double BASE_MULTIPLIER = 0.85;
    double PRECISE_MULTIPLIER = 0.5;

    boolean preciseMovement = false;
    boolean fieldCentric = false;

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

        // PILOT Controls
        pilot.Buttons.Square.onPress(() -> {
            preciseMovement = !preciseMovement;
        });

        pilot.Buttons.Triangle.onPress(() -> {
            fieldCentric = !fieldCentric;
        });

        // COPILOT Controls
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

        /*

        copilot.DPad.Up.onPress(() -> {
            elevator.setLevel(Stack);
        });

        copilot.DPad.Right.onPress(() -> {
            elevator.setLevel(elevator.getJunction(Stack)-Elevator.$());
        });

        copilot.DPad.Left.onPress(() -> {

        });

        copilot.DPad.Down.onPress(() -> {

        });
        */

        // Claw Controls

        copilot.Triggers.Right.onPress(() -> {
            elevator.closeClaw();
        });

        copilot.Triggers.Left.onPress(() -> {
            elevator.openClaw();
        });
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void onLoop() {
        double moveMultiplier = BASE_MULTIPLIER;
        double turnMultiplier = 1;
        if (preciseMovement) {
            moveMultiplier = PRECISE_MULTIPLIER;
            turnMultiplier = PRECISE_MULTIPLIER;
        }
        if (fieldCentric)
            drive.driveType = DriveTrain.DriveType.FIELD_CENTRIC;
        else
            drive.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        drive.gamepadInput = new Vector2d(-gamepad1.left_stick_y * moveMultiplier, -gamepad1.left_stick_x * moveMultiplier);
        drive.gamepadInputTurn = -gamepad1.right_stick_x * turnMultiplier;

        drive.driveTrainPointFieldModes();

        if (copilot.Buttons.Share.isPressed() && copilot.Buttons.Options.isPressed()) {
            elevator.homeElevator();
        }

        elevator.jogElevator(-gamepad2.left_stick_y);

        t.addData("Elevator", elevator.getStateText());
        elevator.showElevatorState();

        Pose2d poseEstimate = drive.getPoseEstimate();
        t.addData("x", "%3.2f", poseEstimate.getX());
        t.addData("y", "%3.2f", poseEstimate.getY());
        t.addData("heading", "%3.2f", Math.toDegrees(drive.getExternalHeading()));
        t.addData("precise", preciseMovement);
        t.addData("drive type", drive.driveType);
        t.update();
    }
}
