package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Calibrate Claw", group = "calibration")
@Config
public class CalibrateClaw extends LinearOpMode {

    public static double claw_closed = 150;
    public static double claw_open = 90;

    ElapsedTime timer;
    SimpleServo claw;

    @Override
    public void runOpMode() throws InterruptedException {

        claw = new SimpleServo(hardwareMap, "claw_servo", 0, 270);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            claw.turnToAngle(claw_closed);

            betterSleep(5000);

            claw.turnToAngle(claw_open);

            betterSleep(5000);
        }
    }

    void betterSleep(long milliseconds) {
        timer = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested() && timer.milliseconds() <= milliseconds) {
            telemetry.addData("angle", claw.getAngle());
            telemetry.update();
        }
    }
}
