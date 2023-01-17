package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetection;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Park")//, preselectTeleOp = "TeleOp")
public class AutoPark extends LinearOpMode {

    private DriveTrain drive;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private MultipleTelemetry t;
    private SleeveDetection.ParkingPosition position;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DriveTrain(hardwareMap);
        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        if (opModeIsActive()) {
            camera.stopStreaming();

            TrajectorySequenceBuilder trajectory = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                    .forward(48);

            switch (position) {
                case LEFT:
                    trajectory.strafeLeft(19);
                    break;
                case RIGHT:
                    trajectory.strafeRight(19);
                    break;
            }

            drive.followTrajectorySequenceAsync(trajectory.build());
            while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
                t.update();
                drive.update();
            }
        }
    }
}