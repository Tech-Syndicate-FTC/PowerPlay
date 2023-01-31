package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.SharedStates;

import java.util.concurrent.TimeUnit;

public class VuforiaSleeve {
    private static final String VUFORIA_KEY =
            "AbNsEgn/////AAABmR5O8hcMQEQPssvwjLBLr5aO0xPBvNfIPkauJEUOdU1NPhwkxRoTmt3jtFWRyZkE1HXTtKDMwzQIDicffaarkkFse+RyfoFRaLz0iT1Ef79arJHWA9DXXNPv+3FKTol0rkrKFpgKDh93gaCOgPOl3HKM3knD9XMhpaBSoyZrFAhDvgMsAuHQOCz5KWrPQ6NXeMhIcMH0QjPJi4bBSmJZ8NrnXkcC3CGuNiKMIohNMTMLmF1N6+ubmo2HTDjYHo4d+EwVRTouIARJEfRmadsB8zJuV98edzTRJiAeuL561B6cfVBV9dTKh0Rk+yu+ED9IQ8BTeCsKgLNgemTqfeu1bKaSypvG9x/eKWuR0sQKCN3s";

    VuforiaLocalizer vuforia = null;
    OpenGLMatrix targetPose = null;
    LinearOpMode myOpMode = null;
    VuforiaTrackables targetsPowerPlay = null;

    public VuforiaSleeve(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        parameters.cameraName = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the trackable objects from the Assets file, and give them meaningful names
        targetsPowerPlay = this.vuforia.loadTrackablesFromAsset("signal");
        targetsPowerPlay.get(0).setName("Signal");

        setupCam((long) SharedStates.cameraExposure, SharedStates.cameraGain);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        // Start tracking targets in the background
        targetsPowerPlay.activate();
    }

    public void setupCam(long exposure, int gain) {
        // Assign the exposure and gain control objects, to use their methods.
        ExposureControl myExposureControl = vuforia.getCamera().getControl(ExposureControl.class);
        GainControl myGainControl = vuforia.getCamera().getControl(GainControl.class);

        myExposureControl.setMode(ExposureControl.Mode.Manual);
        myOpMode.sleep(100);

        // Set initial exposure and gain.
        //myExposureControl.setExposure(10, TimeUnit.MILLISECONDS);
        myExposureControl.setExposure((long) DriveConstants.CAMERA_EXPOSURE, TimeUnit.MILLISECONDS);
        myOpMode.sleep(100);
        myGainControl.setGain(DriveConstants.CAMERA_GAIN);
        myOpMode.sleep(100);
    }

    public int getSignalNumber() {
        int imageNumber = 0;
        // Look for first visible target, and save its pose.
        for (VuforiaTrackable trackable : targetsPowerPlay) {
            myOpMode.telemetry.addData("Tracking", trackable.getName());
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                myOpMode.telemetry.addLine("Target is visible.");
                targetPose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getVuforiaCameraFromTarget();

                // if we have a target, process the "pose" to determine the position of the target relative to the robot.
                if (targetPose != null) {
                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(targetPose, INTRINSIC, ZYX, DEGREES);

                    double imageAngle = normalizeHeading(rotation.secondAngle - 95);

                    String color;


                    if (imageAngle < -60) {
                        imageNumber = 1;
                        color = "Gray";
                    } else if (imageAngle > 60) {
                        imageNumber = 3;
                        color = "Green";
                    } else {
                        imageNumber = 2;
                        color = "Yellow";
                    }

                    SharedStates.parkingSpot = imageNumber;
                    myOpMode.telemetry.addData("Parking Spot", "%d", imageNumber);
                    myOpMode.telemetry.addData("Color", color);
                    break;  // jump out of target tracking loop if we find a target.
                }
            } else {
                myOpMode.telemetry.addLine("Target not visible.");
            }
        }
        myOpMode.telemetry.update();
        return imageNumber;
    }

    public double normalizeHeading(double heading) {
        while (heading <= -180) {
            heading += 360;
        }
        while (heading >= 180) {
            heading -= 360;
        }
        return heading;
    }
}