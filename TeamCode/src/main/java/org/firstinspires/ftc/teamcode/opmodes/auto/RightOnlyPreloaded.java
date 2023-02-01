package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.commands.drive.StrafeCommand.Directions.LEFT;
import static org.firstinspires.ftc.teamcode.commands.drive.StrafeCommand.Directions.RIGHT;
import static org.firstinspires.ftc.teamcode.commands.elevator.ElevatorClawCommand.ClawPositions.CLOSED;
import static org.firstinspires.ftc.teamcode.commands.elevator.ElevatorClawCommand.ClawPositions.OPEN;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.CLOSE_STRAFE;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.FAR_STRAFE;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.FORWARD_DISTANCE;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.STRAFE_DISTANCE;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions.Ground;
import static org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions.High;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.drive.ForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.StrafeCommand;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorClawCommand;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorHomeCommand;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorLevelCommand;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorManualCommand;
import org.firstinspires.ftc.teamcode.subsystems.vision.SleeveDetection;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;

@Autonomous(name = "Right Preloaded Cone Only", preselectTeleOp = "TeleOp")
public class RightOnlyPreloaded extends LinearOpMode {

    private DriveTrain drive;
    private Elevator elevator;
    private SleeveDetection sleeveDetection;
    private boolean finished = false;
    private OpenCvCamera camera;
    private MultipleTelemetry t;
    private SleeveDetection.ParkingPosition position;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DriveTrain(hardwareMap);
        elevator = new Elevator(this, false);
        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sleeveDetect();

        schedule(
                new SequentialCommandGroup(
                        new ElevatorHomeCommand(elevator),
                        new ElevatorClawCommand(elevator, CLOSED),
                        new ForwardCommand(drive, FORWARD_DISTANCE),
                        new ParallelCommandGroup(
                                new ElevatorLevelCommand(elevator, High, 2),
                                new StrafeCommand(drive, STRAFE_DISTANCE, LEFT)
                        ),
                        new WaitCommand(800),
                        new ForwardCommand(drive, 2),
                        new WaitCommand(200),
                        new ElevatorManualCommand(elevator, Elevator.ELEVATOR_HIGH - Elevator.$(5)),
                        new WaitCommand(300),
                        new ElevatorClawCommand(elevator, OPEN),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new ForwardCommand(drive, -2),
                                new ElevatorLevelCommand(elevator, Ground)
                        ),
                        new ElevatorHomeCommand(elevator),
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(SleeveDetection.ParkingPosition.LEFT, new StrafeCommand(drive, CLOSE_STRAFE, LEFT));
                                    put(SleeveDetection.ParkingPosition.CENTER, new StrafeCommand(drive, STRAFE_DISTANCE, RIGHT));
                                    put(SleeveDetection.ParkingPosition.RIGHT, new StrafeCommand(drive, FAR_STRAFE, RIGHT));
                                }},
                                sleeveDetection::getPosition
                        ),
                        new InstantCommand(() -> {
                            finished = true;
                        })
                )
        );

        while (opModeIsActive() && !finished && !isStopRequested()) {
            run();
            drive.update();
            elevator.periodic();

            t.addData("elevator", elevator.getStateText());
            elevator.showElevatorState();

            Pose2d poseEstimate = drive.getPoseEstimate();
            t.addData("pose", "(%3.2f, %3.2f, %3.2f°)", drive.getPoseX(), drive.getPoseY(), drive.getPoseHeading());
            t.update();
        }
        t.addLine("Finished");
        t.update();
    }

    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    public void run() {
        CommandScheduler.getInstance().run();
    }

    public void sleeveDetect() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }
            position = sleeveDetection.getPosition();
            t.addData("position", position);
            t.update();
        }

        waitForStart();
        camera.stopStreaming();
    }
}