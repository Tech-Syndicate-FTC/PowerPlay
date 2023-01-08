package org.firstinspires.ftc.teamcode.subsystems.elevator;

import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.AUTO_GRAB;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.AUTO_RELEASE;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.FLIPPING_UP;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.GOING_HOME_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.HOME_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.HOME_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.HOMING;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.IDLE;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.IN_POSITION_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.IN_POSITION_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.MOVING_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.MOVING_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.RELEASING;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.WAITING_TO_MOVE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.SharedStates;

import java.util.Arrays;
import java.util.List;

public class Elevator {
    // Ticks per inch calculation (537.7*8.7)/38.4=128.8
    private static final double TICKS_PER_IN = 128.8;
    private static final double HEIGHT_OFF_GROUND = 2.5;//5;//2.5;

    final boolean DROP_ON_HOME = true;
    final double HOME_POWER = -0.5;

    public final static int ELEVATOR_MIN = 0;
    public final static int ELEVATOR_HOME = inchesToTicks(2.5);
    public final static int ELEVATOR_STACK_TOP = inchesToTicks(6.5);
    public final static int ELEVATOR_LOW = inchesToTicks(12);
    public final static int ELEVATOR_MID = inchesToTicks(19);
    public final static int ELEVATOR_HIGH = inchesToTicks(26);
    public final static int ELEVATOR_MAX = inchesToTicks(35);

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
    final double SLOW_LIFT = 0.8;
    final double SLOW_LOWER = 0.2;
    final double FAST_LOWER = -0.8;
    final double HOLD_POWER = 0.05;
    final double IN_POSITION_LIMIT = 15;

    final double HAND_HOME_POSITION = 0.3;

    public final double HAND_OPEN = 0.3;
    public final double HAND_READY = 0.4;
    public final double HAND_CLOSE = 0.5;

    private DcMotorEx liftMotor;
    private List<DcMotorEx> motors;
    private Servo hand;   //smaller values open the hand more
    private ElapsedTime elevatorStateTimer = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();

    private LinearOpMode myOpMode = null;
    private boolean isAutonomous = false;

    // elevator state variables
    private int currentElevatorLevel = 0;
    private ElevatorState elevatorState = IDLE;
    private boolean liftActive = false;
    private boolean liftInPosition = false;
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
        liftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "Slide");
        hand = myOpMode.hardwareMap.get(Servo.class, "claw");
        motors = Arrays.asList(liftMotor);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        setHandPosition(HAND_HOME_POSITION);

        currentElevatorLevel = 0;
        newLevelRequested = false;
        elevatorState = SharedStates.elevatorState;

        // do any grabber required initializations
        switch (elevatorState) {
            case HOME_OPEN:
                setHandPosition(HAND_OPEN);
                break;

            case HOME_CLOSED:
                myOpMode.sleep(500);
                setHandPosition(HAND_CLOSE);
                break;

            default:
        }
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
                myOpMode.telemetry.addData("Elevator", "Homing");
                myOpMode.telemetry.update();
                recalibrateHomePosition();
                myOpMode.telemetry.addData("Elevator", "Home Done.");
                myOpMode.telemetry.update();
                if (isAutonomous) {
                    setHandPosition(HAND_CLOSE);
                    setState(HOME_CLOSED);
                } else {
                    setHandPosition(HAND_OPEN);
                    setState(HOME_OPEN);
                }
                break;
            }

            case HOME_OPEN: {
                if (newLiftPosition()) {
                    setLiftTargetPosition(requestedPosition);
                    setState(MOVING_OPEN);
                } else if (grabRequested()) {
                    setHandDelayMove(HAND_CLOSE, 0.3, (liftPosition + 200), HOME_CLOSED);
                } else if (homeRequested()) {
                    setHandPosition(HAND_OPEN);
                    setLiftTargetPosition(ELEVATOR_HOME);
                }
                break;
            }

            case HOME_CLOSED: {
                if (newLiftPosition()) {
                    setLiftTargetPosition(requestedPosition);
                    setState(MOVING_CLOSED);
                } else if (releaseRequested()) {
                    setHandPosition(HAND_OPEN);
                    setState(HOME_OPEN);
                } else if (homeRequested()) {
                    setHandDelayMove(HAND_READY, 0.1, ELEVATOR_HOME, GOING_HOME_OPEN);
                }
                break;
            }

            case GOING_HOME_OPEN: {
                if (liftInPosition) {
                    setHandPosition(HAND_OPEN);
                    currentElevatorLevel = 0;  // Se to home level.
                    setState(HOME_OPEN);
                }
                break;
            }

            case WAITING_TO_MOVE: {
                // wait for timer to elapse then go to new position and state.
                // Get here by calling setWristDelayMove()
                if (elevatorStateTimer.time() >= pendingDelay) {
                    setLiftTargetPosition(pendingLiftPosition);
                    setState(pendingState);
                }
                break;
            }

            case MOVING_OPEN: {
                if (liftInPosition) {
                    setState(IN_POSITION_OPEN);
                } else if (newLiftPosition()) {
                    setLiftTargetPosition(requestedPosition);
                }
                break;
            }

            case IN_POSITION_OPEN: {
                setHandPosition(HAND_OPEN);  // Ensure let go  (should not be required)
                if (homeRequested()) {
                    setHandDelayMove(HAND_READY, 0.1, ELEVATOR_HOME, GOING_HOME_OPEN);
                } else if (grabRequested()) {
                    setHandDelayMove(HAND_CLOSE, 0.3, (liftPosition + 240), IN_POSITION_CLOSED);
                } else if (newLiftPosition()) {
                    setLiftTargetPosition(requestedPosition);
                }

                break;
            }

            case MOVING_CLOSED: {
                if (liftInPosition) {
                    setState(IN_POSITION_CLOSED);
                } else if (newLiftPosition()) {
                    setLiftTargetPosition(requestedPosition);
                }
                break;
            }

            case IN_POSITION_CLOSED: {
                setHandPosition(HAND_CLOSE);  // Hold Tight  (should not be required)

                if (releaseRequested()) {
                    setHandPosition(HAND_OPEN);
                    setState(IN_POSITION_OPEN);
                } else if (newLiftPosition()) {
                    setLiftTargetPosition(requestedPosition);
                    setState(MOVING_CLOSED);
                } else if (homeRequested()) {
                    if (DROP_ON_HOME) {
                        setHandDelayMove(HAND_READY, 0.1, ELEVATOR_HOME, GOING_HOME_OPEN);
                    } else {
                        currentElevatorLevel = 0;  // Se to home level.
                        setLiftTargetPosition(ELEVATOR_HOME);
                        setState(HOME_CLOSED);
                    }
                }
                break;
            }

            // ====  following States ONLY used in Autonomous
            case AUTO_RELEASE: {
                if (liftInPosition) {
                    setHandPosition(HAND_OPEN);
                    //setState(RELEASING);
                    setState(IN_POSITION_OPEN);
                }
                break;
            }

            case RELEASING: {
                if (elevatorStateTimer.time() > 0.25) {
                    terminateSequence = true;  // signal to auto to stop waiting for drop;
                    setState(FLIPPING_UP);
                }
                break;
            }

            case FLIPPING_UP: {
                if (elevatorStateTimer.time() > 0.25) {
                    setLiftTargetPosition(getStackHeight());
                    terminateSequence = true;  // signal to auto to stop waiting for drop;
                    setState(IN_POSITION_OPEN);
                }
                break;
            }

            case AUTO_GRAB: {
                //if (liftInPosition) {
                terminateSequence = true;  // signal to auto to stop waiting for grab;
                setState(IN_POSITION_CLOSED);
                //}
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

    public void setState(ElevatorState newState) {
        if (newState != elevatorState) {
            elevatorState = newState;
            elevatorStateTimer.reset();
            SharedStates.elevatorState = newState;
        }
    }

    public String getStateText() {
        return elevatorState.toString();
    }


    private void setHandDelayMove(double handPosition, double delaySec, int elevatorPosition, ElevatorState nextState) {
        setHandPosition(handPosition);
        pendingDelay = delaySec;
        pendingLiftPosition = elevatorPosition;
        pendingState = nextState;

        setState(WAITING_TO_MOVE);
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
            liftError = liftTargetPosition - getLiftPosition();
            if (liftError > DEAD_BAND * 10) {
                // elevator is way too low
                rampPower(FAST_LIFT);
            } else if (liftError > DEAD_BAND) {


                // elevator is little too low
                setPower(SLOW_LIFT);
            } else if (liftError < -DEAD_BAND * 10) {
                // elevator is way too High
                rampPower(FAST_LOWER);
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

            // Adjust the angle of the servo.
            // first calculate arm angle and negate it for level wrist
            // Then add desired wrist offset angle and send to servo.
            /*
            liftAngle = elevatorEncoderToAngle(liftPosition);
            wristAngle = wristOffset - liftAngle;
            wristPosition = wristAngleToServo(wristAngle);
            setWristPosition(wristPosition);
            */
        }
        return liftInPosition;
    }

    public void showElevatorState() {
        // Display key arm data
        myOpMode.telemetry.addData("arm state", elevatorState);
        myOpMode.telemetry.addData("arm Set/Pos", "%d, %d", liftTargetPosition, liftPosition);
        myOpMode.telemetry.addData("arm Err/Pwr", "%d, %4.2f", liftError, liftMotor.getPower());
    }

    /***
     * Start the power off slowly when moving a long way
     * @param power
     */
    private void rampPower(double power) {
        double currentPower = liftMotor.getPower();
        power = currentPower + ((power - currentPower) * 0.4);
        setPower(power);
    }

    /***
     * Re-learn the elevator home position.
     * Lower the elevator until it stops and then reset zero position.
     */
    public void recalibrateHomePosition() {
        disableLift();  // Stop any closed loop control
        liftLastPosition = liftMotor.getCurrentPosition();
        setPower(HOME_POWER);
        myOpMode.sleep(250);

        while (!myOpMode.isStopRequested() && (liftMotor.getCurrentPosition() != liftLastPosition)) {
            liftLastPosition = liftMotor.getCurrentPosition();
            myOpMode.sleep(100);
        }

        myOpMode.sleep(250);

        setPower(0);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(50);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setLiftTargetPosition(ELEVATOR_HOME);
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

    private boolean homeRequested() {
        return myOpMode.gamepad2.left_bumper;
    }

    // ----- Elevator controls  --- REQUESTS FROM External sources
    //
    public void levelUp() {
        // Move up if not at top.
        if (currentElevatorLevel < ELEVATOR_TOP_LEVEL) {
            // look to see if we are at home, and if hand is open or closed
            if ((currentElevatorLevel == 0) && (handPosition == HAND_CLOSE)) {
                // Jump past Top of Stack
                currentElevatorLevel = 2;
            } else {
                currentElevatorLevel++;
            }
        }
        requestedPosition = elevatorLevel[currentElevatorLevel];

        // If we are picking up from stack, drop the height for next time.
        if (currentElevatorLevel == 1) {
            dropStackHeight();
        }
        newLevelRequested = true;
    }

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
        requestedPosition = newLevel;
        newLevelRequested = true;
    }

    public void openClaw() {
        setHandPosition(HAND_OPEN);
        setState(ElevatorState.IN_POSITION_OPEN);
    }

    public void closeClaw() {
        setHandPosition(HAND_CLOSE);
        setState(ElevatorState.IN_POSITION_CLOSED);
    }

    public void levelDown() {
        // Move down if not at bottom
        if (currentElevatorLevel > 0) {
            currentElevatorLevel--;
        }
        requestedPosition = elevatorLevel[currentElevatorLevel];
        newLevelRequested = true;
    }

    // ===== Autonomous Features   ================================
    // called from AUTO to release cone and make hand safe.
    public void autoRelease() {
        setLiftTargetPosition(Math.max(ELEVATOR_HOME, liftMotor.getCurrentPosition() - ELEVATOR_RELEASE_DROP));
        setState(AUTO_RELEASE);
    }

    // called from AUTO to grab cone and raise to drop on low junction
    public void autoGrab() {
        setHandDelayMove(HAND_CLOSE, 0.3, liftMotor.getCurrentPosition() + ELEVATOR_RELEASE_DROP, AUTO_GRAB);//ELEVATOR_LOW, AUTO_GRAB);
    }

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

    // Local methods to check for a request to grab or release cone.
    private boolean grabRequested() {
        return (driverAutoGrabRequest || driverManualGrabRequest);
    }

    private boolean releaseRequested() {
        return (myOpMode.gamepad2.circle);
    }

    public void enableLift() {
        liftActive = true;
    }

    public void disableLift() {
        liftActive = false;
    }

    public void setLiftTargetPosition(int Position) {
        liftTargetPosition = Range.clip(Position, ELEVATOR_MIN, ELEVATOR_MAX);
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

    public boolean getWristIsSafe() {
        return wristIsSafe;
    }

    public void jogElevator(double speed) {
        setLiftTargetPosition(liftTargetPosition + (int) (speed * 20));
    }

    /*
    public void runElevator(double seconds) {
        runTime.reset();
        while (runTime.time() < seconds) {
            update();
            runStateMachine();
        }
    }
    */

    public void setHandPosition(double position) {
        handPosition = position;
        handIsOpen = (position < HAND_CLOSE);
        hand.setPosition(position);
    }

    // ------- Bulk motor control methods
    public void setPower(double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    // Inches to ticks
    public static int inchesToTicks(double inches) {
        return (int) ((inches - HEIGHT_OFF_GROUND) * TICKS_PER_IN);
    }

    public static double ticksToInches(int ticks) {
        return (double) ((ticks * TICKS_PER_IN) + HEIGHT_OFF_GROUND);
    }
}
