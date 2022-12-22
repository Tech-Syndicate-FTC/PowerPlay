package org.firstinspires.ftc.teamcode.drive.subsystems.drivetrain;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Elevator {
    private static double TICKS_PER_IN = 100;
    private static double MAX_HEIGHT = 35;
    private static int ERROR = 1;
    private static double DROP_AMT = 4;

    public enum Level {
        GROUND,
        REST,
        GROUND_JUNCTION,
        LOW_JUNCTION,
        MID_JUNCTION,
        HIGH_JUNCTION,
    }

    private double[] levelPositions = {
            0,  // Ground
            2,  // Rest
            0,  // Ground Junction
            10, // Low Junction
            20, // Mid Junction
            30  // High Junction
    };

    public enum Direction {
        UP,
        DOWN,
    }

    private DcMotorEx slideMotor;
    private double position = 0;
    private double power = 1;

    private Claw claw;

    public Elevator(HardwareMap hardwareMap, String name) {
        claw = new Claw(hardwareMap, "claw");

        slideMotor = hardwareMap.get(DcMotorEx.class, name);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setTargetPositionTolerance(inToTicks(ERROR));

        //position = slideMotor.getCurrentPosition();
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setTargetPosition(inToTicks(position));
    }

    public void update() {
        slideMotor.setTargetPosition(inToTicks(position));
        slideMotor.setPower(power);

        claw.update();
    }

    public boolean atTargetPosition() {
        return position - ERROR <= slideMotor.getCurrentPosition() && slideMotor.getCurrentPosition() >= position + ERROR;
    }

    public void waitUntilAtTarget() {
        while (!atTargetPosition()) {}
        return;
    }

    public void dropCone() {
        slideAsync(Direction.DOWN, DROP_AMT);
        claw.open();
    }

    public void pickCone() {
        claw.close();
        slideAsync(Direction.UP, DROP_AMT);
    }

    public void slide(Level level) {
        int levelIdx = 0;

        switch (level) {
            case GROUND:
                levelIdx = 0;
                break;
            case REST:
                levelIdx = 1;
                break;
            case GROUND_JUNCTION:
                levelIdx = 2;
                break;
            case LOW_JUNCTION:
                levelIdx = 3;
                break;
            case MID_JUNCTION:
                levelIdx = 4;
                break;
            case HIGH_JUNCTION:
                levelIdx = 5;
                break;
        }
        position = levelPositions[levelIdx];
        position = Range.clip(position, 0, MAX_HEIGHT);
    }

    public void slide(Direction direction, double pos) {
        switch (direction) {
            case UP:
                position += pos;
                break;
            case DOWN:
                position -= pos;
                break;
        }
        position = Range.clip(position, 0, MAX_HEIGHT);
    }

    public void slideAsync(Direction dir, double pos) {
        slide(dir, pos);
        waitUntilAtTarget();
    }

    public void slideAsync(Level level) {
        slide(level);
        waitUntilAtTarget();
    }

    private int inToTicks(double inches) {
        return (int) (TICKS_PER_IN * inches);
    }
}
