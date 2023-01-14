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

@Autonomous(name = "Auto Park")
public class AutoPark extends LinearOpMode {

    private DriveTrain drive;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private MultipleTelemetry t;

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
            t.addData("Position", sleeveDetection.getPosition());
            t.update();
        }
        waitForStart();
        if (opModeIsActive()) {
            camera.stopStreaming();

            TrajectorySequenceBuilder traj = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                    .forward(50);

            switch (sleeveDetection.getPosition()) {
                case LEFT:
                    traj.strafeLeft(12);
                case RIGHT:
                    traj.strafeRight(12);
            }

            drive.followTrajectorySequenceAsync(traj.build());
            while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
                for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                    module.clearBulkCache();
                }
                t.update();
                drive.update();
            }
        }
    }
}