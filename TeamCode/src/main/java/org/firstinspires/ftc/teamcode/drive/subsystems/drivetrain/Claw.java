package org.firstinspires.ftc.teamcode.drive.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Claw {
    static final double CLOSE_POS = 0.5;     // Maximum rotational position
    static final double OPEN_POS = 0.3;     // Minimum rotational position
    private double position = CLOSE_POS;

    private Servo hand;

    public Claw(HardwareMap hardwareMap, String name) {
        hand = hardwareMap.get(Servo.class, name);
    }

    public void update() {
        position = Range.clip(position, OPEN_POS, CLOSE_POS);
        hand.setPosition(position);
    }

    public void open() {
        position = OPEN_POS;
    }

    public void close() {
        position = CLOSE_POS;
    }
}
