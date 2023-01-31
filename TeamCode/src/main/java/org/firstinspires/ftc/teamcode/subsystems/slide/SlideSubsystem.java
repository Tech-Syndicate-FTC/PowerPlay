package org.firstinspires.ftc.teamcode.subsystems.slide;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlideSubsystem {

    public STATE state = STATE.GOOD;

    public static final double MAX_EXTENSION = 37.75;
    public final double TICKS_PER_IN = 81.8;

    public final MotorEx slideLeft;
    public final MotorEx slideRight;
    public final Motor.Encoder slideEncoder;
    public final SimpleServo claw;

    public double claw_pos_open = 81;
    public double claw_pos_closed = 148.5;

    public MotionProfile profile;
    public MotionState curState;
    private final ElapsedTime timer;
    private PIDFController controller;

    public boolean isExtended = false;

    private double position;

    public static double P = 0.06;
    public static double I = 0.013;
    public static double D = 0.0;
    public static double F = 0.0001;

    public double power = 0.0;
    public double targetPosition = 0.0;

    public double pivotOffset = 0.0;

    public enum STATE {
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }

    public enum ClawState {
        OPEN,
        CLOSED,
    }

    public enum ArmState {
        INTAKE,
        TRANSITION,
        DEPOSIT,
        SCORE
    }

    public enum PivotState {
        FLAT, PITCH_UP, SCORE, DOWN
    }

    public SlideSubsystem(HardwareMap hardwareMap, boolean isAuto) {
        this.slideLeft = new MotorEx(hardwareMap, "slide_left");
        this.slideRight = new MotorEx(hardwareMap, "slide_right");
        this.claw = new SimpleServo(hardwareMap, "claw_servo", 0, 270);
        slideRight.setInverted(true);
        this.slideEncoder = slideLeft.encoder;
        if (isAuto) {
            slideEncoder.reset();
        }

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), 30, 25);

        this.timer = new ElapsedTime();
        timer.reset();
        this.controller = new PIDFController(P, I, D, F);
    }

    public void update(ClawState state) {
        switch (state) {
            case OPEN:
                claw.turnToAngle(claw_pos_open);
                break;
            case CLOSED:
                claw.turnToAngle(claw_pos_closed);
                break;
        }
    }

    public void loop() {
        controller = new PIDFController(P, I, D, F);

        curState = profile.get(timer.time());
        if (curState.getV() != 0) {
            targetPosition = curState.getX();
        }

        isExtended = (getPos() > TICKS_PER_IN * 2.5);

        power = controller.calculate(position, targetPosition);
    }

    public void read() {
        position = slideEncoder.getPosition();
    }

    public void write() {
        slideRight.set(power);
        slideLeft.set(power);
    }


    public void setSlideFactor(double factor) {
        double slideAddition = TICKS_PER_IN * factor;
        double newPosition = position + slideAddition;
        if (curState.getV() == 0 && newPosition >= 0 && newPosition <= MAX_EXTENSION * TICKS_PER_IN) {
            targetPosition = newPosition;
        }
    }

    public int getPos() {
        return (int) position;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new com.acmerobotics.roadrunner.profile.MotionState(getPos(), 0), new com.acmerobotics.roadrunner.profile.MotionState(targetPos, 0), max_v, max_a);
        resetTimer();
    }
}
