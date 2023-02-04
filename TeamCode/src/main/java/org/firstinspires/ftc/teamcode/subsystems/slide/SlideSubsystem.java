package org.firstinspires.ftc.teamcode.subsystems.slide;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlideSubsystem {

    public STATE state = STATE.GOOD;

    public static final double MAX_EXTENSION_IN = 37.5;
    public static final double MAX_EXTENSION = 3040;
    public static final double TICKS_PER_IN = MAX_EXTENSION / MAX_EXTENSION_IN;

    public final MotorEx slideLeft;
    public final MotorEx slideRight;
    //public final Motor.Encoder slideEncoder;
    public final SimpleServo claw;
    public final SimpleServo armLeft;
    public final SimpleServo armRight;
    public final SimpleServo clawPivot;

    public MotionProfile profile;
    public MotionState curState;
    private final ElapsedTime timer;
    private PIDFController controller;

    public boolean isExtended = false;

    public double leftPosition;
    public double rightPosition;

    public static double P = 0.01;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;

    public double leftPower = 0.0;
    public double rightPower = 0.0;
    public double targetPosition = 0.0;

    public double armAngle = 0.0;

    public double pivotOffset = 0.0;

    public boolean isAuto = false;

    public enum STATE {
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT,
        FAILED
    }

    public enum ClawState {
        OPEN(90),
        CLOSED(158);
        double targetAngle;

        ClawState(double angle) {
            this.targetAngle = angle;
        }
    }

    public enum ArmState {
        INTAKE(0),
        TRANSITION(120),
        SCORE_LOW(115),
        UPRIGHT(160),
        AIM(170),
        SCORE(220);

        double targetAngle;

        ArmState(double angle) {
            this.targetAngle = angle;
        }
    }

    public enum PivotState {
        FLAT(0),
        AIM(60);

        double targetOffset;

        PivotState(double offset) {
            this.targetOffset = offset;
        }
    }

    public SlideSubsystem(HardwareMap hardwareMap, boolean isAuto) {
        this.slideLeft = new MotorEx(hardwareMap, "slide_left");
        this.slideRight = new MotorEx(hardwareMap, "slide_right");
        slideLeft.setRunMode(Motor.RunMode.RawPower);
        slideRight.setRunMode(Motor.RunMode.RawPower);
        slideRight.setInverted(true);
        slideLeft.resetEncoder();
        slideRight.resetEncoder();

        this.claw = new SimpleServo(hardwareMap, "claw_servo", 0, 300);
        this.armLeft = new SimpleServo(hardwareMap, "arm_pivot_left", 0, 300);
        this.armLeft.setInverted(true);
        this.armRight = new SimpleServo(hardwareMap, "arm_pivot_right", 0, 300);
        this.clawPivot = new SimpleServo(hardwareMap, "claw_pivot", 0, 300);

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0), new MotionState(0, 0), 30, 25);

        this.timer = new ElapsedTime();
        timer.reset();
        this.controller = new PIDFController(P, I, D, F);

        this.isAuto = isAuto;

        update(ClawState.OPEN);
    }

    public void update(ClawState state) {
        claw.turnToAngle(state.targetAngle);
    }

    public void update(ArmState state) {
        armAngle = state.targetAngle;
    }

    public void update(PivotState state) {
        pivotOffset = state.targetOffset;
    }

    public void loop() {
        controller = new PIDFController(P, I, D, F);

        curState = profile.get(timer.time());
        if (curState.getV() != 0) {
            targetPosition = curState.getX();
        }

        isExtended = (getPos() > TICKS_PER_IN * 2.5);

        leftPower = controller.calculate(leftPosition, targetPosition);
        rightPower = controller.calculate(rightPosition, targetPosition);
    }

    public void read() {
        leftPosition = slideLeft.getCurrentPosition();
        rightPosition = slideRight.getCurrentPosition();
    }

    public void write() {
        slideRight.set(rightPower);
        slideLeft.set(leftPower);
        positionArm(armAngle);
    }

    public void periodic() {
        read();
        loop();
        write();
    }

    public void positionArm(double angle) {
        clawPivot.turnToAngle(angle + pivotOffset);
        armLeft.turnToAngle(angle + 1);
        armRight.turnToAngle(angle);
    }

    public void setSlideFactor(double factor) {
        double slideAddition = TICKS_PER_IN * factor;
        double newPosition = getPos() + slideAddition;
        if (curState.getV() == 0 && newPosition >= 0 && newPosition <= MAX_EXTENSION) {
            targetPosition = newPosition;
        }
    }

    public int getPos() {
        return (int) leftPosition;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new com.acmerobotics.roadrunner.profile.MotionState(getPos(), 0), new com.acmerobotics.roadrunner.profile.MotionState(targetPos, 0), max_v, max_a);
        resetTimer();
    }
}
