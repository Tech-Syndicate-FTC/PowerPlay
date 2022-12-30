package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "Test Autonomous", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    DriveTrain driveTrain;
    //Telemetry telemetry;
    ElapsedTime runTime;

    public int parkingSpot = 1;

    public enum START_POSITION {
        LEFT,
        RIGHT,
    }

    public START_POSITION startPosition = START_POSITION.RIGHT;

    static TrajectorySequence trajectoryAuto;

    static double tile = 24;
    static double halfTile = tile / 2;

    //Initialize any other Pose2d's as desired
    Pose2d initPose = relativePose(1.5 * tile, -2.5 * tile, 90);
    Pose2d avoidSignalPose = relativePose(0.5 * tile, -2.5 * tile, 90);
    Pose2d dropPreloadedConePose = relativePose(0.5 * tile, -tile, 90);
    Pose2d postPreloadedDropPose = relativePose(halfTile - 3, -halfTile, 90);
    Pose2d pickConePose = relativePose(2.5 * tile - 7, -halfTile + 1, 0);
    Pose2d midWayPose = relativePose(1.5 * tile - 3, -halfTile, 90);
    Pose2d dropConePose = relativePose(1.5 * tile - 4, -halfTile + 6, 135);
    Pose2d parkPose;

    public Pose2d relativePose(double x, double y, double heading) {
        if (startPosition == START_POSITION.LEFT) {
            x = -x;
            heading = 180 - heading;
        }
        return new Pose2d(x, y, Math.toRadians(heading));
    }

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        int coneAmt = 1;

        buildParking();

        TrajectorySequenceBuilder tmpTrj = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(avoidSignalPose)
                .lineToLinearHeading(dropPreloadedConePose)
                .addDisplacementMarker(() -> dropCone(0))
                .lineToLinearHeading(postPreloadedDropPose)
                .lineToLinearHeading(midWayPose);

        for (int i = 1; i <= coneAmt; i++) {
            // Workaround to prevent error from variable having to be final
            int currentCone = i;
            tmpTrj = tmpTrj
                    .lineToLinearHeading(pickConePose)
                    .waitSeconds(1)
                    .addDisplacementMarker(() -> pickCone(currentCone))
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(dropConePose)
                    .waitSeconds(1)
                    .addDisplacementMarker(() -> dropCone(currentCone))
                    .lineToLinearHeading(midWayPose);
        }

        trajectoryAuto = tmpTrj
                .lineToLinearHeading(parkPose)
                .build();
    }

    public void buildParking() {
        switch (startPosition) {
            case RIGHT:
                switch (parkingSpot) {
                    case 1:
                        parkPose = new Pose2d(0.5 * tile, -halfTile - 2, Math.toRadians(270));
                        break;
                    case 2:
                        parkPose = new Pose2d(1.5 * tile, -halfTile - 2, Math.toRadians(270));
                        break;
                    case 3:
                        parkPose = new Pose2d(2.5 * tile - 4, -halfTile - 2, Math.toRadians(270));
                        break;
                }
                break;
            case LEFT:
                switch (parkingSpot) {
                    case 1:
                        parkPose = new Pose2d(-2.5 * tile + 4, -halfTile - 2, Math.toRadians(270));
                        break;
                    case 2:
                        parkPose = new Pose2d(-1.5 * tile, -halfTile - 2, Math.toRadians(270));
                        break;
                    case 3:
                        parkPose = new Pose2d(-0.5 * tile, -halfTile - 2, Math.toRadians(270));
                        break;
                }
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        driveTrain = new DriveTrain(hardwareMap);

        waitForStart();

        //if (opModeIsActive() && !isStopRequested()) {
        //Build parking trajectory based on last detected target by vision
        runTime = new ElapsedTime();
        buildAuto();
        driveTrain.getLocalizer().setPoseEstimate(initPose);
        driveTrain.followTrajectorySequence(trajectoryAuto);
        //}

        //Trajectory is completed, display Parking complete
        parkingComplete();
    }

    public void parkingComplete() {
        telemetry.addData("Parked in Location", parkingSpot);
        telemetry.addData("Run Time:", runTime.seconds());
        telemetry.update();
    }

    public void pickCone(int coneCount) {
        /*TODO: Add code to pick Cone 1 from stack*/
        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(int coneCount) {
        /*TODO: Add code to drop cone on junction*/
        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }
}
