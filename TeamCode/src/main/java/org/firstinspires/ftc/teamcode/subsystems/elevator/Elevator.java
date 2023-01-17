package org.firstinspires.ftc.teamcode.subsystems.elevator;

import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.HOME;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.HOMING;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.IDLE;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.IN_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.MOVING;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.WAITING_TO_MOVE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.SharedStates;

import java.util.Arrays;
import java.util.List;

public class Elevator {
    // Ticks per inch calculation (537.7*8.7)/38.4=128.8
    private static final double TICKS_PER_IN = 128.8;
    private static final double HEIGHT_OFF_GROUND = 2.5;

    final boolean DROP_ON_HOME = true;
    final double HOME_POWER = -0.5;

    public final static int ELEVATOR_MIN = 0;
    public final static int ELEVATOR_HOME = 0;
    public final static int ELEVATOR_STACK_TOP = inchesToTicks(6.5);
    public final static int ELEVATOR_LOW = inchesToTicks(12);
    public final static int ELEVATOR_MID = inchesToTicks(19);
    public final static int ELEVATOR_HIGH = inchesToTicks(28);
    public final static int ELEVATOR_MAX = inchesToTicks(37);

    final static int ELEVATOR_COUNTS_PER_CONE = inchesToTicks(1);
    final static int ELEVATOR_RELEASE_DROP = inchesToTicks(7);

    final static int ELEVATOR_TOP_LEVEL = 5;
    final static int elevatorLevel[] = {
            Elevator.ELEVATOR_MIN,
            Elevator.ELEVATOR_HOME,
            Elevator.ELEVATOR_STACK_TOP,
            Elevator.ELEVATOR_LOW,
            Elevator.ELEVATOR_MID,
            Elevator.ELEVATOR_HIGH
    };

    public enum Junctions {
        Ground,
        Low,
        Medium,
        High
    }

    final int DEAD_BAND = 20;
    final double FAST_LIFT = 1;
    final double SLOW_LIFT = 0.2;
    final double SLOW_LOWER = -0.2;
    final double FAST_LOWER = -0.7;
    final double HOLD_POWER = 0.05;
    final double IN_POSITION_LIMIT = 10;

    final double HAND_HOME_POSITION = 0.3;

    // Hand Values!!!
    public final double HAND_OPEN = 0.3;
    public final double HAND_CLOSE = 0.55; //higher is more closed
    public final double HAND_READY = HAND_OPEN + (HAND_CLOSE - HAND_OPEN) / 2;

    private MotorEx liftMotor;
    private Servo hand;
    private ElapsedTime elevatorStateTimer = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();

    private LinearOpMode myOpMode = null;
    private boolean isAutonomous = false;

    // elevator state variables
    private int currentElevatorLevel = 0;
    private ElevatorState elevatorState = IDLE;
    private boolean liftActive = false;
    public boolean liftInPosition = false;
    private int liftPosition = 0;
    private int liftError = 0;
    private int liftTargetPosition = 0;
    private int liftLastPosition = 0;
    private boolean wristIsSafe = false;

    public boolean newLevelRequested = false;
    private int requestedPosition;
    private double pendingDelay;
    private int pendingLiftPosition;
    private ElevatorState pendingState;

    public boolean driverAutoGrabRequest = false;
    public boolean driverManualGrabRequest = false;
    public boolean terminateSequence = false;
    public boolean handIsOpen = true;

    private double handPosition = 0;

    public Elevator(LinearOpMode opMode, boolean isAuto) {
        // Attach to hardware devices
        myOpMode = opMode;
        isAutonomous = isAuto;
        liftMotor = new MotorEx(myOpMode.hardwareMap, "Slide", Motor.GoBILDA.RPM_435);//myOpMode.hardwareMap.get(DcMotorEx.class, "Slide");
        hand = myOpMode.hardwareMap.get(Servo.class, "claw");

        liftMotor.setRunMode(Motor.RunMode.PositionControl);
        liftMotor.setTargetPosition(liftTargetPosition);
        liftMotor.set(0);
        liftMotor.setPositionTolerance(IN_POSITION_LIMIT);

        //liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        if (isAuto) {
            setHandPosition(HAND_CLOSE);
        } else {
            setHandPosition(HAND_HOME_POSITION);
        }

        currentElevatorLevel = 0;
        newLevelRequested = false;
        elevatorState = SharedStates.elevatorState;
    }

    public Elevator(BaseOpMode opMode, boolean isAuto) {
        this((LinearOpMode) opMode, isAuto);
    }

    // ======  Elevator State Machine
    public ElevatorState runStateMachine(ElevatorState newState) {
        setState(newState);
        switch (elevatorState) {
            case IDLE: {
                resetStackHeight();
                setState(HOMING);
                break;
            }

            case HOMING: {
                myOpMode.telemetry.addLine("lift homing");
                myOpMode.telemetry.update();
                recalibrateHomePosition();
                myOpMode.telemetry.addLine("homing finished");
                myOpMode.telemetry.update();
                setState(IN_POSITION);
                break;
            }

            case MOVING: {
                if (liftInPosition) {
                    setState(IN_POSITION);
                } else if (newLiftPosition()) {
                    setLiftTargetPosition(requestedPosition);
                }
                break;
            }

            case IN_POSITION: {
                if (newLiftPosition()) {
                    setLiftTargetPosition(requestedPosition);
                }

                break;
            }
        }
        return elevatorState;
    }

    /*
    ----------PILOT/CO-PILOT-------------
    ----------ELEVATOR CONTROLS----------
     */

    /***
     * This is the normal way to run the Elevator state machine.
     * Only use the parameter form if you want to force a state change
     * @return current state.
     */
    public ElevatorState runStateMachine() {
        return runStateMachine(elevatorState);
    }

    public void periodic() {
        update();
        runStateMachine();
    }

    public void setState(ElevatorState newState) {
        if (newState != elevatorState) {
            elevatorState = newState;
            elevatorStateTimer.reset();
            SharedStates.elevatorState = newState;
        }
    }

    public ElevatorState getState() {
        return elevatorState;
    }

    public String getStateText() {
        return elevatorState.toString();
    }

    /**
     * Perform closed loop control on elevator and wrist
     *
     * @return true if lift is In Position
     */
    public boolean update() {
        liftPosition = liftMotor.getCurrentPosition();

        // Run the elevator motor with 4 different speed zones.  Two up and two down.
        if (liftActive) {
            /*
            liftError = liftTargetPosition - getLiftPosition();
            if (liftError > DEAD_BAND * 10) {
                // elevator is way too low
                setPower(FAST_LIFT);
            } else if (liftError > DEAD_BAND) {
                // elevator is little too low
                setPower(SLOW_LIFT);
            } else if (liftError < -DEAD_BAND * 10) {
                // elevator is way too High
                setPower(FAST_LOWER);
            } else if (liftError < -DEAD_BAND) {
                // elevator is little too High
                setPower(SLOW_LOWER);
            } else {
                // We are in position, so apply a little hold power unless we are at rest on support
                if (liftTargetPosition <= ELEVATOR_HOME)
                    setPower(0);
                else
                    setPower(HOLD_POWER);
            }

            liftInPosition = (Math.abs(liftError) <= IN_POSITION_LIMIT);
            */
            if (!liftMotor.atTargetPosition()) {
                liftMotor.set(1);
                liftInPosition = false;
            } else {
                liftMotor.stopMotor();
                liftInPosition = true;
            }

            hand.setPosition(handPosition);
        }
        return liftInPosition;
    }

    public void showElevatorState() {
        // Display key arm data
        myOpMode.telemetry.addData("lift state", elevatorState);
        myOpMode.telemetry.addData("lift target/reality", "%d, %d", liftTargetPosition, liftPosition);
        myOpMode.telemetry.addData("lift error/power", "%d", liftError);
        myOpMode.telemetry.addData("lift in position", liftInPosition);
    }

    public void homeElevator() {
        setState(IDLE);
    }

    /***
     * Re-learn the elevator home position.
     * Lower the elevator until it stops and then reset zero position.
     */
    public void recalibrateHomePosition() {
        disableLift();  // Stop any closed loop control
        liftMotor.setRunMode(Motor.RunMode.RawPower);
        liftLastPosition = liftMotor.getCurrentPosition();
        liftMotor.set(HOME_POWER);
        myOpMode.sleep(250);

        while (!myOpMode.isStopRequested() && (liftMotor.getCurrentPosition() != liftLastPosition)) {
            liftLastPosition = liftMotor.getCurrentPosition();
            myOpMode.sleep(50);
        }

        myOpMode.sleep(50);
        liftMotor.set(0);
        myOpMode.sleep(250);

        liftMotor.resetEncoder();
        liftMotor.setRunMode(Motor.RunMode.PositionControl);
        setLiftTargetPosition(0);
        currentElevatorLevel = 0;
        newLevelRequested = false;
        enableLift();  // Start closed loop control
    }

    private boolean newLiftPosition() {
        if (newLevelRequested) {
            newLevelRequested = false;
            return true;
        } else {
            return false;
        }
    }

    // ----- Elevator controls  --- REQUESTS FROM External sources

    public void setLevel(Junctions junction) {
        int newLevel = 0;
        switch (junction) {
            case Ground:
                newLevel = ELEVATOR_MIN;
                break;
            case Low:
                newLevel = ELEVATOR_LOW;
                break;
            case Medium:
                newLevel = ELEVATOR_MID;
                break;
            case High:
                newLevel = ELEVATOR_HIGH;
                break;
        }
        setLiftTargetPosition(newLevel);
    }

    public void setLevel(int target) {
        setLiftTargetPosition(target);
    }


    public void openClaw() {
        setHandPosition(HAND_OPEN);
    }

    public void closeClaw() {
        setHandPosition(HAND_CLOSE);
    }

    // ===== Autonomous Features   ================================
    // called from AUTO to release cone and make hand safe.

    public boolean sequenceComplete() {
        if (terminateSequence) {
            terminateSequence = false;
            return true;
        } else {
            return false;
        }
    }


    // ===== Autonomous Features   ================================

    public void dropStackHeight() {
        elevatorLevel[1] -= ELEVATOR_COUNTS_PER_CONE;
    }

    public int getStackHeight() {
        return elevatorLevel[1];
    }

    public void resetStackHeight() {
        elevatorLevel[1] = ELEVATOR_STACK_TOP;
    }

    public void enableLift() {
        liftActive = true;
    }

    public void disableLift() {
        liftActive = false;
    }

    public void setLiftTargetPosition(int Position) {
        liftTargetPosition = Range.clip(Position, ELEVATOR_MIN, ELEVATOR_MAX);
        liftMotor.setTargetPosition(liftTargetPosition);
        liftInPosition = false;  // set this now to prevent the state machine from skipping
    }

    public int getLiftTargetPosition() {
        return liftTargetPosition;
    }

    public int getLiftPosition() {
        return liftPosition;
    }

    public int getLiftRawPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void jogElevator(double speed) {
        setLiftTargetPosition(liftTargetPosition + (int) (speed * 20));
    }

    public void setHandPosition(double position) {
        handPosition = position;
        handIsOpen = (position < HAND_CLOSE);
        hand.setPosition(position);
    }

    // Inches to ticks
    public static int inchesToTicks(double inches) {
        return (int) ((inches - HEIGHT_OFF_GROUND) * TICKS_PER_IN);
    }

    public static double ticksToInches(int ticks) {
        return (double) ((ticks * TICKS_PER_IN) + HEIGHT_OFF_GROUND);
    }
}
