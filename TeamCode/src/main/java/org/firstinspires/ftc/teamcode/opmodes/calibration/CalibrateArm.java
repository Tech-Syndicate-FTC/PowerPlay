package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Calibrate Arm", group = "calibration")
@Config
public class CalibrateArm extends LinearOpMode {
    SimpleServo clawPivot;
    SimpleServo armLeft;
    SimpleServo armRight;

    public static double homeAngle = 180;
    public static double groundAngle = 0;
    public static double uprightAngle = 140;
    public static double transitionAngle = 90;
    public static double armAngle = transitionAngle;

    @Override
    public void runOpMode() throws InterruptedException {
        armLeft = new SimpleServo(hardwareMap, "arm_pivot_left", 0, 300);
        armLeft.setInverted(true);
        armRight = new SimpleServo(hardwareMap, "arm_pivot_right", 0, 300);
        clawPivot = new SimpleServo(hardwareMap, "claw_pivot", 0, 270);
        //clawPivot.setInverted(true);

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            clawPivot.turnToAngle(armAngle);
            sleep(1000);
            armLeft.turnToAngle(armAngle + 1);
            armRight.turnToAngle(armAngle);
        }

        waitSeconds(2);
        armAngle = uprightAngle;
        waitSeconds(2);
        armAngle = groundAngle;
        waitSeconds(2);
        armAngle = transitionAngle;
        waitSeconds(2);
        armAngle = uprightAngle;
        waitSeconds(2);
        armAngle = homeAngle;
        waitSeconds(2);

    }

    void waitSeconds(long seconds) {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested() && timer.milliseconds() <= seconds * 1000) {
            clawPivot.turnToAngle(armAngle);
            armLeft.turnToAngle(armAngle + 1);
            armRight.turnToAngle(armAngle);
        }
    }
}
