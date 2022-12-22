package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class MeepMeepTesting {

    static int parkingSpot = 3;

    public enum START_POSITION {
        LEFT,
        RIGHT,
    }

    public static START_POSITION startPosition = START_POSITION.RIGHT;

    static TrajectorySequence trajectoryAuto;
    static TrajectorySequence trajectoryParking;

    static double tile = 24;
    static double halfTile = tile / 2;

    //Initialize any other Pose2d's as desired
    static Pose2d initPose = relativePose(1.5 * tile, -2.5 * tile, 90);
    static Pose2d avoidSignalPose = relativePose(0.5 * tile, -2.5 * tile, 90);
    static Pose2d dropPreloadedConePose = relativePose(0.5 * tile, -tile, 180);
    static Pose2d postPreloadedDropPose = relativePose(halfTile, -halfTile, 90);
    static Pose2d pickConePose = relativePose(2.5 * tile - 4, -halfTile, 0);
    static Pose2d midWayPose = relativePose(1.5 * tile, -halfTile, 90);
    static Pose2d dropConePose = relativePose(32, -8, 135);
    static Pose2d parkPose;

    public static Pose2d relativePose(double x, double y, double heading) {
        if (startPosition == START_POSITION.LEFT) {
            x = -x;
            heading = 180 - heading;
        }
        return new Pose2d(x, y, Math.toRadians(heading));
    }

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public static void buildAuto(DriveShim drive) {
        int coneAmt = 3;

        buildParking();

        TrajectorySequenceBuilder tmpTrj = drive.trajectorySequenceBuilder(initPose)
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
                    .addDisplacementMarker(() -> pickCone(currentCone))
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(dropConePose)
                    .addDisplacementMarker(() -> dropCone(currentCone))
                    .lineToLinearHeading(midWayPose);
        }

        trajectoryAuto = tmpTrj
                .lineToLinearHeading(parkPose)
                .build();
    }

    public static void buildParking() {
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

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive -> {
                    buildAuto(drive);
                    return trajectoryAuto;
                });

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static void dropCone(int coneCount) {
        if (coneCount == 0) {
            //System.out.println("Dropped Cone: Pre-loaded");
        } else {
            //System.out.println(String.format("Dropped Cone: Stack %d", coneCount));
        }
    }

    public static void pickCone(int coneCount) {
        //System.out.println(String.format("Picked Cone: Stack %d", coneCount));
    }
}