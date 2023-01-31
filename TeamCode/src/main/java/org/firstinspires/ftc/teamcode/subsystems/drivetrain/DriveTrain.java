package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain extends MecanumBase {

    public DriveMode driveMode = DriveMode.NORMAL;
    public DriveType driveType = DriveType.FIELD_CENTRIC;
    // For Position
    public Pose2d poseEstimate = new Pose2d(0, 0, 0);
    public Vector2d gamepadInput = new Vector2d(0, 0);
    public double gamepadInputTurn = 0;

    /**
     * A parameter to register all the hardware devices for DriveTrain
     *
     * @param hardwareMap Hardware Map
     */
    public DriveTrain(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void update() {
        poseEstimate = getPoseEstimate();
        super.update();
    }

    /**
     * Main drive modes implemented here
     * - Robot or Field centric selection
     * - Precise control to be used in TeleOp
     */
    public void gamepadDrive() {
        poseEstimate = getPoseEstimate();

        switch (driveMode) {
            case NORMAL:
            default:
                gamepadInput = new Vector2d(gamepadInput.getX() * 0.85, gamepadInput.getY() * 0.85);
            case PRECISION:
                gamepadInput = new Vector2d(gamepadInput.getX() * 0.5, gamepadInput.getY() * 0.5);
                gamepadInputTurn = gamepadInputTurn * 0.5;
        }

        switch (driveType) {
            case ROBOT_CENTRIC:
                setWeightedDrivePower(new Pose2d(gamepadInput.getX(), gamepadInput.getY(), gamepadInputTurn));
                break;
            case FIELD_CENTRIC:
                Vector2d input = new Vector2d(gamepadInput.getX(), gamepadInput.getY()).rotated(-getExternalHeading());
                setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), gamepadInputTurn));
        }

        getLocalizer().update();
    }

    public enum DriveMode {
        NORMAL, PRECISION;

        public DriveMode toggle() {
            if (this == NORMAL) return PRECISION;
            else return NORMAL;
        }
    }

    public enum DriveType {
        ROBOT_CENTRIC, FIELD_CENTRIC;

        public DriveType toggle() {
            if (this == ROBOT_CENTRIC) return FIELD_CENTRIC;
            else return ROBOT_CENTRIC;
        }
    }
}
