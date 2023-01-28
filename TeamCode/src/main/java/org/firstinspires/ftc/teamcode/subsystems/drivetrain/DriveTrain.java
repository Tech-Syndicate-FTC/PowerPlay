package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.DriveConstants;

public class DriveTrain extends MecanumBase {

    public enum DriveType {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
    }

    public DriveType driveType = DriveType.FIELD_CENTRIC;

    // For Position
    public Pose2d poseEstimate = new Pose2d(0, 0, 0);

    /**
     * A parameter to register all the hardware devices for DriveTrain
     *
     * @param hardwareMap
     */
    public DriveTrain(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void update() {
        super.update();
    }

    public Vector2d gamepadInput = new Vector2d(0, 0);
    public double gamepadInputTurn = 0;

    /**
     * Main drive modes implemented here
     * - Robot or Field centric selection
     * - Augmented control to be used in TeleOp
     * - Turn to Center lie (0 deg)
     * - Delta turn by 5 degrees (left and right)
     */
    public void driveTrainPointFieldModes() {
        poseEstimate = getPoseEstimate();
        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection = new Pose2d();

        switch (driveType) {
            case ROBOT_CENTRIC:
                driveDirection = new Pose2d(
                        gamepadInput.getX(),
                        gamepadInput.getY(),
                        gamepadInputTurn
                );
                break;
            case FIELD_CENTRIC:
                Vector2d input = new Vector2d(
                        gamepadInput.getX(),
                        gamepadInput.getY()
                ).rotated(-getExternalHeading());
                driveDirection = new Pose2d(
                        input.getX(),
                        input.getY(),
                        gamepadInputTurn
                );
        }


        setWeightedDrivePower(driveDirection);

        getLocalizer().update();
    }
}
