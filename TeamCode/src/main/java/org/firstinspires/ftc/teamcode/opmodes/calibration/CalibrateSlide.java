package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.slide.SlideSubsystem;

@Autonomous(name = "Calibrate Slide")
public class CalibrateSlide extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double MAX_EXTENSION = SlideSubsystem.MAX_EXTENSION;

        MotorEx slide = new MotorEx(hardwareMap, "slide_left");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            int position = slide.getCurrentPosition();
            telemetry.addData("position", position);
            telemetry.addData("TICKS_PER_IN", position / MAX_EXTENSION);
            telemetry.update();
        }
    }
}
