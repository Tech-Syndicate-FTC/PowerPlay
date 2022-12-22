package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;
import java.util.List;

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
    static Pose2d initPose = new Pose2d(1.5 * tile, -2.5 * tile, Math.toRadians(90));
    static Pose2d midPose = new Pose2d(0.5 * tile, -2.5 * tile, Math.toRadians(90));
    static Pose2d dropPreConePose = new Pose2d(0.5 * tile, -tile, Math.toRadians(180)); //Choose the pose to move to the sxtack of cones
    static Pose2d partWayPose = new Pose2d(halfTile, -halfTile, Math.toRadians(90));
    static Pose2d pickConePose = new Pose2d(2.5 * tile - 4, -halfTile, Math.toRadians(0)); //Choose the pose to move to the stack of cones
    static Pose2d midWayPose = new Pose2d(1.5 * tile, -halfTile, Math.toRadians(90));
    //static Pose2d midWayPose = new Pose2d(tile, -halfTile, Math.toRadians(90));
    static Pose2d dropConePose = new Pose2d(32, -8, Math.toRadians(135)); //Choose the pose to move to the stack of cones
    //static Pose2d dropConePose = new Pose2d(tile, -halfTile + 1, Math.toRadians(90)); //Choose the pose to move to the stack of cones
    static Pose2d parkPose; //= new Pose2d(12, -12, Math.toRadians(270));

    static List<Pose2d> poses = Arrays.asList(initPose, midPose, dropPreConePose, partWayPose, midWayPose, pickConePose, dropConePose);

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public static void buildAuto(DriveShim drive) {
        boolean flipSide = false;
        if (startPosition == START_POSITION.LEFT) {
            flipSide = true;
        }

        buildParking();

        for (int i = 0; i < poses.size(); i++) {
            Pose2d pose = poses.get(i);
            double x = pose.getX();
            double y = pose.getY();
            double heading = Math.toDegrees(pose.getHeading());
            if (flipSide) {
                x = -x;
                heading = 180 - heading;
            }
            poses.set(i, new Pose2d(x, y, Math.toRadians(heading)));
        }

        //Drop Preloaded Cone, Pick 5 cones and park
        trajectoryAuto = drive.trajectorySequenceBuilder(poses.get(0))
                //.lineToLinearHeading(midWayPose)
                //Uncomment following line to slow down turn if needed.
                //.setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, TRACK_WIDTH))
                .lineToLinearHeading(poses.get(1))
                .lineToLinearHeading(poses.get(2))
                .addDisplacementMarker(() -> {
                    dropCone(0); //Drop preloaded Cone
                })
                //Uncomment following line to stop reduction in speed. And move to the position after which you want to stop reducing speed.
                //.resetVelConstraint()
                .lineToLinearHeading(poses.get(3))
                .lineToLinearHeading(poses.get(4))
                .lineToLinearHeading(poses.get(5))
                .addDisplacementMarker(() -> {
                    pickCone(1); //Pick top cone from stack
                })
                .lineToLinearHeading(poses.get(4))
                .lineToLinearHeading(poses.get(6))
                .addDisplacementMarker(() -> {
                    dropCone(1); //Drop cone on junction
                })
                .lineToLinearHeading(poses.get(4))
                .lineToLinearHeading(poses.get(5))
                .addDisplacementMarker(() -> {
                    pickCone(2); //Pick second cone from stack
                })
                .lineToLinearHeading(poses.get(4))
                .lineToLinearHeading(poses.get(6))
                .addDisplacementMarker(() -> {
                    dropCone(2); //Drop cone on junction
                })
                .lineToLinearHeading(poses.get(4))
                .lineToLinearHeading(parkPose)
                .build();
    }

    public static void buildParking() {
        switch (startPosition) {
            case RIGHT:
                switch (parkingSpot) {
                    case 1:
                        parkPose = new Pose2d(0.5 * tile, -halfTile - 2, Math.toRadians(270));
                        break; // Location 1
                    case 2:
                        parkPose = new Pose2d(1.5 * tile, -halfTile - 2, Math.toRadians(270));
                        break; // Location 2
                    case 3:
                        parkPose = new Pose2d(2.5 * tile - 4, -halfTile - 2, Math.toRadians(270));
                        break; // Location 3
                }
                break;
            case LEFT:
                switch (parkingSpot) {
                    case 1:
                        parkPose = new Pose2d(-2.5 * tile + 4, -halfTile - 2, Math.toRadians(270));
                        break; // Location 1
                    case 2:
                        parkPose = new Pose2d(-1.5 * tile, -halfTile - 2, Math.toRadians(270));
                        break; // Location 2
                    case 3:
                        parkPose = new Pose2d(-0.5 * tile, -halfTile - 2, Math.toRadians(270));
                        break; // Location 3
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
        /*TODO: Add code to drop cone on junction*/
        if (coneCount == 0) {
            //System.out.println("Dropped Cone: Pre-loaded");
        } else {
            //System.out.println(String.format("Dropped Cone: Stack %d", coneCount));
        }
    }

    public static void pickCone(int coneCount) {
        /*TODO: Add code to pick Cone 1 from stack*/
        //System.out.println(String.format("Picked Cone: Stack %d", coneCount));
    }
}