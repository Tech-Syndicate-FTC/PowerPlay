package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystems.SharedStates;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.function.Consumer;
import java.util.function.Supplier;

@Autonomous(name = "Vision Setup", group = "Autonomous")
public class VisionSetup extends BaseOpMode {
    Vision vision;

    @Override
    public void init() {
        vision = new Vision(this.opMode);

        pilot.Buttons.Triangle.onPress(() -> {
            SharedStates.cameraExposure += 10;
            setupCam();
        });
        pilot.Buttons.X.onPress(() -> {
            SharedStates.cameraExposure += 10;
            setupCam();
        });

        pilot.Buttons.Square.onPress(() -> {
            SharedStates.cameraGain += 10;
            setupCam();
        });
        pilot.Buttons.Circle.onPress(() -> {
            SharedStates.cameraGain += 10;
            setupCam();
        });

        telemetry.addLine("Y / Δ Exposure +");
        telemetry.addLine("A / X Exposure -");
        telemetry.addLine("X / ▢ Gain +");
        telemetry.addLine("B / O Gain -");
        telemetry.addLine();
        telemetry.addData("Exposure: %d", SharedStates.cameraExposure);
        telemetry.addData("Gain: %d", SharedStates.cameraGain);
        telemetry.update();
    }

    @Override
    public void initLoop() {
        vision.getSignalNumber();
        telemetry.addLine("Y / Δ Exposure +");
        telemetry.addLine("A / X Exposure -");
        telemetry.addLine("X / ▢ Gain +");
        telemetry.addLine("B / O Gain -");
        telemetry.addLine();
        telemetry.addData("Exposure: %d", SharedStates.cameraExposure);
        telemetry.addData("Gain: %d", SharedStates.cameraGain);
        telemetry.update();
    }

    void setupCam() {
        vision.setupCam((long) SharedStates.cameraExposure, SharedStates.cameraGain);
    }
}
