package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Calibrate Arm", group = "calibration")
@Config
public class CalibrateArm extends LinearOpMode {
    ElapsedTime timer;
    SimpleServo clawPivot;
    SimpleServo armLeft;
    SimpleServo armRight;

    public static double clawAngle = 0;
    public static double armAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        armLeft = new SimpleServo(hardwareMap, "arm_pivot_left", 0, 300);
        armLeft.setInverted(true);
        armRight = new SimpleServo(hardwareMap, "arm_pivot_right", 0, 300);
        clawPivot = new SimpleServo(hardwareMap, "claw_pivot", 0, 270);
        clawPivot.setInverted(true);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            armLeft.setPosition(armAngle + 0.05);
            armRight.setPosition(armAngle);
            clawPivot.setPosition(clawAngle);
        }

    }

    void moveArm(double angle) {
        //clawPivot.turnToAngle(-angle - pivotOffset);
        //armPivot.turnToAngle(angle);
    }
}
