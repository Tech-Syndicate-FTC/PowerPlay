package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.slide.SlideSubsystem;

@Autonomous(name = "Calibrate Slide", group = "calibration")
public class CalibrateSlide extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx slide = new MotorEx(hardwareMap, "slide_left");

        slide.resetEncoder();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            int position = slide.getCurrentPosition();
            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}
