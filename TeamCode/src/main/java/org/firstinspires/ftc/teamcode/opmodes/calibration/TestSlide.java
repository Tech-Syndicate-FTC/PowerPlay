package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.helpers.gamepad.PS4Buttons;

@TeleOp(name = "Test Slide", group = "calibration")
@Config
public class TestSlide extends LinearOpMode {

    public static final double MAX_EXTENSION_IN = 37.5;
    public static final double MAX_EXTENSION = 3040;
    public static final double TICKS_PER_IN = MAX_EXTENSION / MAX_EXTENSION_IN;
    public static double P = 0.01;

    public static double pos1 = 0;
    public static double pos2 = MAX_EXTENSION / 3;
    public static double pos3 = MAX_EXTENSION / 3 * 2;
    public static double pos4 = MAX_EXTENSION;

    public MotionProfile profile;
    public MotionState curState;

    private ElapsedTime timer;

    public double position = 0.0;
    public static int target = 0;

    public static double MAX_V = 6000;
    public static double MAX_A = 5000;

    private double voltage;
    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    public static boolean voltageModeration = false;

    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx slideLeft = new MotorEx(hardwareMap, "slide_left");
        slideLeft.setRunMode(Motor.RunMode.RawPower);
        MotorEx slideRight = new MotorEx(hardwareMap, "slide_right");
        slideRight.setRunMode(Motor.RunMode.RawPower);
        slideRight.setInverted(true);

        PIDController controller = new PIDController(P, 0, 0);

        slideLeft.resetEncoder();

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0), new MotionState(0, 0), MAX_V, MAX_A);

        this.timer = new ElapsedTime();
        timer.reset();

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();

        waitForStart();

        GamepadEx copilot = new GamepadEx(gamepad2);

        copilot.getGamepadButton(PS4Buttons.TRIANGLE).whenPressed(() -> {
            newProfile(pos4, MAX_V, MAX_A);
        });

        copilot.getGamepadButton(PS4Buttons.SQUARE).whenPressed(() -> {
            newProfile(pos3, MAX_V, MAX_A);
        });

        copilot.getGamepadButton(PS4Buttons.CIRCLE).whenPressed(() -> {
            newProfile(pos2, MAX_V, MAX_A);
        });

        copilot.getGamepadButton(PS4Buttons.CROSS).whenPressed(() -> {
            newProfile(pos1, MAX_V, MAX_A);
        });

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            position = slideLeft.getCurrentPosition();
            target = Range.clip(target, 0, (int) (MAX_EXTENSION));

            if (voltageTimer.seconds() > 5) {
                voltage = voltageSensor.getVoltage();
                voltageTimer.reset();
            }

            curState = profile.get(timer.time());
            if (curState.getV() != 0) {
                target = (int) curState.getX();
            }

            double power = controller.calculate(position, target) / (voltageModeration ? voltage * 14 : 1);

            slideRight.set(power);
            slideLeft.set(power);

            telemetry.addData("target/reality", "%d, %d", target, (int) position);
            telemetry.update();
        }
    }

    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new com.acmerobotics.roadrunner.profile.MotionState(getPos(), 0), new com.acmerobotics.roadrunner.profile.MotionState(targetPos, 0), max_v, max_a);
        resetTimer();
    }

    public int getPos() {
        return (int) position;
    }
}
