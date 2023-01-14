package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;

public abstract class BaseOpMode extends LinearOpMode{
    public MultipleTelemetry t;
    public DriveTrain drive;
    public Elevator elevator;
    public GamepadEx pilot;
    public GamepadEx copilot;

    public void runOpMode() throws InterruptedException {
        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        elevator = new Elevator(this, false);
        pilot = new GamepadEx(gamepad1);
        copilot = new GamepadEx(gamepad1);
        drive = new DriveTrain(hardwareMap);
        onInit();
        while (!isStarted()) {
            onInitLoop();
        }
        waitForStart();
        onStart();
        while (!isStopRequested() && opModeIsActive()) {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }
            pilot.poll(gamepad1);
            copilot.poll(gamepad2);
            onLoop();
            drive.update();
            elevator.update();
            elevator.runStateMachine();
        }
        onStop();
    }

    public void onInit() {
    }

    public void onInitLoop() {
    }

    public void onStart() {
    }

    public void onLoop() {
    }

    public void onStop() {
    }
}
