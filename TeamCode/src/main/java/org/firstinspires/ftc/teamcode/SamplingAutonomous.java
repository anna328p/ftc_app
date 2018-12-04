package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Autonomous Sampling", group = "Concept")

//public class ConceptTensorFlowObjectDetection2 extends LinearOpMode {
public class SamplingAutonomous extends EncoderAutonomous {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    protected ElapsedTime runtime = new ElapsedTime();

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *	  ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "ATPJ7K3/////AAABmfILUtO9kUH0pGVq+86Mck4V5/1Q+SlqSyg/qv6gFItx2SGMZn0WMtMwzIcAb22aAXKisje6GWLXUyIu/HO3Z2NlI4yxs8LE70QuauuTgRYgISfUPlXPSDxU8pBOtZtFaiF+EPlJapjGkiMvmhm5J1QngW2VCvcw0x6Wg6oE8Zy+LX4AhZrztvmNYH0gDns312QPGOmGNdfhU46yRD2qEBzSfBprfAciDhZ2GD7eVtZAG+SWKDJNAfAtWpySnKaERm+ZKCzHZ6DgrwxHBvMAeTdv2Eqe4T2klJ410jljjrGdWJ0P4B6rVLwygZISwotbGoioBTFncBCpX1Oe121Av2Tp+XX5GLrlyCod432hcLdj";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    protected int targetPos;

    @Override
    public void runOpMode() {
        telemetry.addData("***", "0");
        initVuforia();
        telemetry.addData("***", "2");
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addData("***", "3");
        /** Wait for the game to begin */
	/*	telemetry.addData(">", "Press Play to start tracking");
		telemetry.update();
		telemetry.addData("***","4");*/
        waitForStart();

        super.runOpMode();

        robot.flagServo.setPosition(0.8);

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setTargetPosition(10500); //5.5 in
        robot.liftMotor.setPower(1.0);
        telemetry.addData("lifter", robot.liftMotor.getCurrentPosition());
        telemetry.update();
        while (robot.liftMotor.isBusy()) {
            telemetry.addData("lifter", robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }

        robot.liftMotor.setPower(0.0);

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        targetPos = 0; //-1 = left, 0 = center, 1 = right
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 3.0) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData("Mineral:", recognition.getLabel());
                    }
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (updatedRecognitions.size() == 2) {
                        if (silverMineral1X != -1 && silverMineral2X != -1) {
                            targetPos = 1;
                        } else if (silverMineral1X != -1 && silverMineral1X < goldMineralX) {
                            targetPos = 0;
                        } else if (silverMineral1X != -1 && silverMineral1X > goldMineralX) {
                            targetPos = -1;
                        } else if (silverMineral2X != -1 && silverMineral2X < goldMineralX) {
                            targetPos = 0;
                        } else if (silverMineral2X != -1 && silverMineral2X > goldMineralX) {
                            targetPos = -1;
                        }
                        telemetry.addData("Mineral Position", targetPos);
                    } else if (updatedRecognitions.size() == 3) {
                        if (goldMineralX > silverMineral2X && goldMineralX > silverMineral1X) {
                            targetPos = 1;
                        } else if (goldMineralX < silverMineral2X && goldMineralX < silverMineral1X) {
                            targetPos = -1;
                        } else {
                            targetPos = 0;
                        }
                        telemetry.addData("Mineral Position", targetPos);
                    }
                }
            }
            telemetry.update();
        }

        telemetry.addData("Mineral Position Final", targetPos);
        telemetry.addData("Motor Status",
                (robot.leftFrontMotor == null) + " " +
                        (robot.leftBackMotor == null) + " " +
                        (robot.rightFrontMotor == null) + " " +
                        (robot.rightBackMotor == null));
        telemetry.update();

        encoderDrive(TURN_SPEED, -5, 5, 4.0);
        encoderDrive(DRIVE_SPEED, 6, 6, 5);

        if (targetPos == 1) {
            encoderDrive(TURN_SPEED, 11, -11, 5);
            encoderDrive(DRIVE_SPEED, 30, 30, 5);
        } else if (targetPos == 0) {
            encoderDrive(TURN_SPEED, 5, -5, 5);
            encoderDrive(DRIVE_SPEED, 26, 26, 3);
        } else if (targetPos == -1) { //left
            encoderDrive(TURN_SPEED, -2, 2, 4);
            encoderDrive(DRIVE_SPEED, 24, 24, 5);
        }
        telemetry.update();

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    //private void initVuforia() {
    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     */
    //	VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    //	parameters.vuforiaLicenseKey = VUFORIA_KEY;
    //	parameters.cameraDirection = CameraDirection.BACK;

    //  Instantiate the Vuforia engine
    //	vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    //}
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "ATPJ7K3/////AAABmfILUtO9kUH0pGVq+86Mck4V5/1Q+SlqSyg/qv6gFItx2SGMZn0WMtMwzIcAb22aAXKisje6GWLXUyIu/HO3Z2NlI4yxs8LE70QuauuTgRYgISfUPlXPSDxU8pBOtZtFaiF+EPlJapjGkiMvmhm5J1QngW2VCvcw0x6Wg6oE8Zy+LX4AhZrztvmNYH0gDns312QPGOmGNdfhU46yRD2qEBzSfBprfAciDhZ2GD7eVtZAG+SWKDJNAfAtWpySnKaERm+ZKCzHZ6DgrwxHBvMAeTdv2Eqe4T2klJ410jljjrGdWJ0P4B6rVLwygZISwotbGoioBTFncBCpX1Oe121Av2Tp+XX5GLrlyCod432hcLdj";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
