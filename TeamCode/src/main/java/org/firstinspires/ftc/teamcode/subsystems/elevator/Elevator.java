package org.firstinspires.ftc.teamcode.subsystems.elevator;

import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.HOMING;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.IDLE;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState.IN_POSITION;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.SharedStates;

public class Elevator {
    // Ticks per inch calculation (537.7*8.7)/38.4=128.8
    private static final double TICKS_PER_IN = 86.1979166;//87.11328125;

    final double HOME_POWER = -0.5;

    public final static int ELEVATOR_MIN = 0;
    public final static int ELEVATOR_HOME = 0;
    public final static int ELEVATOR_STACK_TOP = $(5);
    public final static int ELEVATOR_LOW = $(12);
    public final static int ELEVATOR_MID = $(21);
    public final static int ELEVATOR_HIGH = $(35);
    public final static int ELEVATOR_MAX = $(38.4);


    public enum Junctions {
        Ground,
        Low,
        Medium,
        High
    }

    final double POSITION_TOLERANCE = 15;

    final double HAND_HOME_POSITION = 0.3;

    // Hand Values!!!
    public final double HAND_OPEN = 0.3;
    public final double HAND_CLOSE = 0.55; //higher is more closed
    public final double HAND_READY = HAND_OPEN + (HAND_CLOSE - HAND_OPEN) / 2;

    private DcMotorEx liftMotor;
    private Servo hand;
    private ElapsedTime elevatorStateTimer = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();

    private LinearOpMode myOpMode = null;
    private boolean isAutonomous = false;

    // elevator state variables
    private int currentElevatorLevel = 0;
    private ElevatorState elevatorState = IDLE;
    private boolean liftActive = SharedStates.elevatorHomed;
    public boolean liftInPosition = false;
    private int liftPosition = 0;
    private int liftError = 0;
    private int liftTargetPosition = 0;
    private int liftLastPosition = 0;

    public boolean newLevelRequested = false;
    private int requestedPosition;

    public boolean handIsOpen = true;

    private double handPosition = 0;

    private PIDFController controller = new PIDFController(5, 0, 0, 0.8);

    public Elevator(LinearOpMode opMode, boolean isAuto) {
        // Attach to hardware devices
        myOpMode = opMode;
        isAutonomous = isAuto;
        liftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "Slide");
        hand = myOpMode.hardwareMap.get(Servo.class, "claw");

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        controller.setTolerance(POSITION_TOLERANCE);


        if (isAuto) {
            setHandPosition(HAND_CLOSE);
        } else {
            setHandPosition(HAND_HOME_POSITION);
        }

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

        liftError = liftTargetPosition - getLiftPosition();

        liftInPosition = Math.abs(liftError) <= POSITION_TOLERANCE;

        if (!liftInPosition) {
            double error = controller.calculate(liftPosition, liftTargetPosition);
            liftMotor.setVelocity(error);
        }
        return liftInPosition;
    }

    public void setPower(double power) {
        liftMotor.setPower(power);
    }

    public void showElevatorState() {
        // Display key arm data
        myOpMode.telemetry.addData("lift state", elevatorState);
        myOpMode.telemetry.addData("lift target/reality", "%d, %d", liftTargetPosition, liftPosition);
        myOpMode.telemetry.addData("lift error/power", "%d, %4.2f", liftError, liftMotor.getPower());
        myOpMode.telemetry.addData("lift in position", liftInPosition);
    }

    public void homeElevator() {
        setState(HOMING);
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
            myOpMode.sleep(50);
        }

        setPower(0);
        myOpMode.sleep(150);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(50);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setLiftTargetPosition(ELEVATOR_HOME);
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


    // ===== Autonomous Features   ================================

    public void enableLift() {
        liftActive = true;
        SharedStates.elevatorHomed = true;
    }

    public void disableLift() {
        liftActive = false;
        SharedStates.elevatorHomed = false;
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

    public int getLiftError() {
        return liftError;
    }

    public int getAbsLiftError() {
        return Math.abs(liftError);
    }

    public int getLiftRawPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void jogElevator(double speed) {
        //setLiftTargetPosition(liftTargetPosition + (int) (speed * 20));
        setLiftTargetPosition(liftTargetPosition + $(speed));
    }

    public void setHandPosition(double position) {
        handPosition = position;
        handIsOpen = (position < HAND_CLOSE);
        hand.setPosition(position);
    }

    // Inches to ticks
    public static int $(double inches) {
        return (int) (inches * TICKS_PER_IN);
    }
}
