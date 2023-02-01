package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain extends MecanumBase {

    public DrivePrecision drivePrecision = DrivePrecision.NORMAL;
    public DriveType driveType = DriveType.FIELD_CENTRIC;
    // For Position
    public Pose2d poseEstimate = new Pose2d(0, 0, 0);
    public Vector2d gamepadInput = new Vector2d(0, 0);
    public double gamepadInputTurn = 0;
    public double manualPrecision = 0;

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

        switch (drivePrecision) {
            case NORMAL:
            default:
                gamepadInput = new Vector2d(gamepadInput.getX() * 0.9, gamepadInput.getY() * 0.9);
                break;
            case PRECISION:
                gamepadInput = new Vector2d(gamepadInput.getX() * 0.5, gamepadInput.getY() * 0.5);
                gamepadInputTurn = gamepadInputTurn * 0.5;
                break;
            case MANUAL:
                double speedMultiplier = 1 - 0.75 * manualPrecision;
                gamepadInput = new Vector2d(gamepadInput.getX() * speedMultiplier, gamepadInput.getY() * speedMultiplier);
                gamepadInputTurn = gamepadInputTurn * speedMultiplier;
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

    public double getPoseX() {
        return getPoseEstimate().getX();
    }

    public double getPoseY() {
        return getPoseEstimate().getY();
    }

    public double getPoseHeading() {
        return getPoseHeading(AngleUnit.DEGREES);
    }

    public double getPoseHeading(AngleUnit unit) {
        if (unit == AngleUnit.RADIANS) return getExternalHeading();
        else if (unit == AngleUnit.DEGREES) return Math.toDegrees(getExternalHeading());
        return getExternalHeading();
    }

    public enum DrivePrecision {
        NORMAL, PRECISION, MANUAL;

        public DrivePrecision toggle() {
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
