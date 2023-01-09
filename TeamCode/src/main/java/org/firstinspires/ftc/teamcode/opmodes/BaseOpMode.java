package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;

public class BaseOpMode {
    public MultipleTelemetry telemetry;
    public HardwareMap hardwareMap;
    public DriveTrain drive;
    public Elevator elevator;
    public GamepadEx pilot;
    public GamepadEx copilot;
    public LinearOpMode opMode = new LinearOpMode() {
        @Override
        public void runOpMode() throws InterruptedException {

        }
    };

    final public void runOpMode() throws InterruptedException {
        opMode.runOpMode();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardwareMap = opMode.hardwareMap;
        elevator = new Elevator(opMode, false);
        pilot = new GamepadEx(opMode.gamepad1);
        copilot = new GamepadEx(opMode.gamepad1);
        drive = new DriveTrain(hardwareMap);
        init();
        while (opMode.opModeInInit()) {
            initLoop();
        }
        once();
        while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }
            pilot.poll(opMode.gamepad1);
            copilot.poll(opMode.gamepad2);
            loop();
            drive.update();
            elevator.update();
            elevator.runStateMachine();
        }
    }

    public void init() {
    }

    public void initLoop() {
    }

    public void once() {
    }

    public void loop() {
    }

    public void sleep(long ms) {
        opMode.sleep(ms);
    }
}
