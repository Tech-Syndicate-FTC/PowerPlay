package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Autonomous(name = "Vision Test", group = "Autonomous")
public class VisionTest extends LinearOpMode {
    Vision vision;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(this);
        vision.init();

        // Show configuration menu, and update trajectories if any changes.
        while (opModeInInit()) {
            vision.getSignalNumber();
        }

        while (opModeIsActive() && !isStopRequested()) {
        }
    }
}
