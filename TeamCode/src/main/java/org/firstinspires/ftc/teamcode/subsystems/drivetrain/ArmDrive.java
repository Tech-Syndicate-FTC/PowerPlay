package org.firstinspires.ftc.teamcode.drive.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class ArmDrive {

    static final double CLAW_INCREMENT = 0.03;
    static final double CLAW_MAX_POS = 0.5;     // Maximum rotational position
    static final double CLAW_MIN_POS = 0.3;     // Minimum rotational position
    private double clawPosition = CLAW_MAX_POS; // Start at halfway position
    private Servo servo;

    private DcMotorEx slide;

    static final double SLIDE_POWER_UP = 0.7;
    static final double SLIDE_POWER_DOWN = 0.5;
    static final double SLIDE_POWER_REST = 0.1;
    static final double SLIDE_INCREMENT = 100;
    private int slidePosition = 0;
    private double slidePower = 0;

    public ArmDrive(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "claw");
        slide = hardwareMap.get(DcMotorEx.class, "Slide");

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setTargetPosition(slide.getCurrentPosition());
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidePosition = slide.getTargetPosition();
        update();
    }

    public void update() {
        clawPosition = Range.clip(clawPosition, CLAW_MIN_POS, CLAW_MAX_POS);
        servo.setPosition(clawPosition);

        slide.setTargetPosition(slidePosition);
        slide.setPower(1);
    }

    /*
        Claw Controls
     */

    public void closeClawManual() {
        clawPosition -= CLAW_INCREMENT;
    }

    public void openClawManual() {
        clawPosition += CLAW_INCREMENT;
    }

    public void closeClaw() {
        clawPosition = CLAW_MAX_POS;
    }

    public void openClaw() {
        clawPosition = CLAW_MIN_POS;
    }

    public double getClawPosition() {
        return servo.getPosition() * 100;
    }

    /*
        Slide Controls
     */

    public void slideUp() {
        slidePosition += SLIDE_INCREMENT;
        //slidePower = SLIDE_POWER_UP;
    }

    public void slideDown() {
        slidePosition -= SLIDE_INCREMENT;
        //slidePower = SLIDE_POWER_DOWN;
    }

    public void restSlide() {
        //slidePower = SLIDE_POWER_REST;
    }

    public void setSlidePosition(int pos) {
        slidePosition = pos;
    }

    public double getSlidePosition() {
        return slide.getCurrentPosition();
    }

    public double getSlidePower() {
        return slide.getPower();
    }
}
