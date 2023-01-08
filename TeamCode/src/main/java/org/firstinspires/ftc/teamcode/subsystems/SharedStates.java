package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.IDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState;

public class SharedStates {
    /**
     * Simple static field serving as a storage medium for the bot's pose.
     * This allows different classes/opmodes to set and read from a central source of truth.
     * A static field allows data to persist between opmodes.
     */
    public static Pose2d currentPose = null;
    public static ElevatorState elevatorState = IDLE;
    public static int parkingSpot = 0;
    public static int cameraGain = 160;
    public static int cameraExposure = 20;
}