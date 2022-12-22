package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Adopted from ConceptTensorFlowObjectDetection.java in external.examples
 *
 * IMPORTANT: In order to use this, you need to obtain your own Vuforia license key as
 * is explained below.
 */
public class Vision {

    public int identifiedparkingLocation = 1; //1 for Location 1, 2 for Location 2 and 3 for Location 3

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    //TODO : Create VUFORIA_KEY for your team based on above instructions and update
    private static final String VUFORIA_KEY =
            "AbNsEgn/////AAABmR5O8hcMQEQPssvwjLBLr5aO0xPBvNfIPkauJEUOdU1NPhwkxRoTmt3jtFWRyZkE1HXTtKDMwzQIDicffaarkkFse+RyfoFRaLz0iT1Ef79arJHWA9DXXNPv+3FKTol0rkrKFpgKDh93gaCOgPOl3HKM3knD9XMhpaBSoyZrFAhDvgMsAuHQOCz5KWrPQ6NXeMhIcMH0QjPJi4bBSmJZ8NrnXkcC3CGuNiKMIohNMTMLmF1N6+ubmo2HTDjYHo4d+EwVRTouIARJEfRmadsB8zJuV98edzTRJiAeuL561B6cfVBV9dTKh0Rk+yu+ED9IQ8BTeCsKgLNgemTqfeu1bKaSypvG9x/eKWuR0sQKCN3s";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    public Vision(HardwareMap hardwareMap) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first.
        initVuforia();
        initTfod(hardwareMap);
    }

    public void activateVuforiaTensorFlow() {


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.25, 16.0 / 9.0);
        }
    }

    public void runVuforiaTensorFlow() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

                    switch (recognition.getLabel()) {
                        case "1 Bolt":
                            identifiedparkingLocation = 1;
                            break;
                        case "2 Bulb":
                            identifiedparkingLocation = 2;
                            break;
                        case "3 Panel":
                            identifiedparkingLocation = 3;
                            break;
                    }
                }
            }
        }
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    /**
     * Stop Tensor Flow algorithm
     */
    public void deactivateVuforiaTensorFlow(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }

}