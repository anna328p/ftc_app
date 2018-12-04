/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@TeleOp(name = "Concept: TensorFlow Object Detection + Crater Autonomous", group = "Concept")

//public class ConceptTensorFlowObjectDetection2 extends LinearOpMode {
public class ConceptTensorFlowObjectDetection2 extends EncoderAutonomous {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    /*	protected static final double	 COUNTS_PER_MOTOR_REV	= 1120 ;	// eg: TETRIX Motor Encoder
        protected static final double	 DRIVE_GEAR_REDUCTION	= 1 ;	 // This is < 1.0 if geared UP
        protected static final double	 WHEEL_DIAMETER_INCHES   = 4.0 ;	 // For figuring circumference
        protected static final double	 COUNTS_PER_INCH		 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                          (WHEEL_DIAMETER_INCHES * 3.1415);
        protected static final double	 DRIVE_SPEED			 = 0.5;
        protected static final double	 TURN_SPEED			  = 0.3;*/
    //protected RobotHardware		 robot   = new RobotHardware();   // Use a Pushbot's hardware
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


    @Override
    public void runOpMode() {
        telemetry.addData("***", "0");
        super.runOpMode();
        telemetry.addData("***", "1");
        //robot.init(hardwareMap);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        telemetry.addData("***", "2");
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addData("***", "3");
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        telemetry.addData("***", "4");
        waitForStart();
	/*	robot.collectorJoint1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.collectorJoint1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.collectorJoint1.setTargetPosition((int)(1680 * 1.2));
		robot.collectorJoint1.setPower(1.0);
		sleep (3000);
		robot.collectorJoint1.setPower(-0.005);
		
		robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.liftMotor.setTargetPosition(-11000); //5.5 in
		robot.liftMotor.setPower(-1.0);
		telemetry.addData("lifter", robot.liftMotor.getCurrentPosition());
		telemetry.update();
		while (robot.liftMotor.isBusy()) {
			telemetry.addData("lifter", robot.liftMotor.getCurrentPosition());
			telemetry.update();
		}
	
		robot.liftMotor.setPower(0.0);*/

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData("Mineral:", recognition.getLabel());
                        }
                        if (updatedRecognitions.size() == /*3*/ 2) {
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
						  /*if (silverMineral1X == -1) {
							silverMineral1X = (int) recognition.getLeft();
						  } else {
							silverMineral2X = (int) recognition.getLeft();
						  }
						}*/
						/*if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
						  if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
							telemetry.addData("Gold Mineral Position", "Left");
						  } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
							telemetry.addData("Gold Mineral Position", "Right");
						  } else {
							telemetry.addData("Gold Mineral Position", "Center");
						  }
						}*/
					/*	if (silverMineral1X != -1 && silverMineral2X != -1){
							telemetry.addData("Gold Mineral Position", "Right");
						  		
						}
						else if (goldMineralX != -1 && silverMineral1X != -1) {
						  if (goldMineralX > silverMineral1X) {
							telemetry.addData("Gold Mineral Position", "Left");
						  } else if (silverMineral1X > goldMineralX) {
							telemetry.addData("Gold Mineral Position", "Center");
							encoderDrive(DRIVE_SPEED, 6, 6, 5);
							encoderDrive(TURN_SPEED, 8, -8, 5);
							encoderDrive(DRIVE_SPEED, 45, 45, 5);
							telemetry.addData("Path", "Complete");
							telemetry.update();
						  } */
                            if (silverMineral1X != -1 && silverMineral2X != -1) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                encoderDrive(TURN_SPEED, -5, 5, 4.0);
                                encoderDrive(DRIVE_SPEED, 6, 6, 5);
                                encoderDrive(TURN_SPEED, 10, -16, 5);
                                encoderDrive(DRIVE_SPEED, 45, 45, 5);
                                telemetry.addData("Path", "Complete");
                                telemetry.update();
                            } else if (silverMineral1X != -1) {
                                telemetry.addData("SilverMineral1X", silverMineral1X);
                                telemetry.addData("GoldMineralX", goldMineralX);
                                if (silverMineral1X > goldMineralX) {
						  		/*telemetry.addData("SilverMineral1X", silverMineral1X);
						  		telemetry.addData("GoldMineralX", goldMineralX);*/
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    encoderDrive(TURN_SPEED, -5, 5, 4.0);
                                    encoderDrive(DRIVE_SPEED, 6, 6, 3);
                                    encoderDrive(TURN_SPEED, 5, -8, 3);
                                    encoderDrive(DRIVE_SPEED, 25, 25, 3);
                                    encoderDrive(DRIVE_SPEED, -12, -12, 3);
                                    encoderDrive(TURN_SPEED, -13, 13, 3);
                                    encoderDrive(DRIVE_SPEED, 40, 40, 3);
                                    encoderDrive(TURN_SPEED, 20, -20, 3);
                                    encoderDrive(10, -40, -40, 4);
                                    robot.flagServo.setPosition(0.1);
                                    sleep(4000);
                                    robot.flagServo.setPosition(0.8);
                                    encoderDrive(10, 80, 80, 4);
                                    telemetry.addData("Path", "Complete");
                                    telemetry.update();
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Left1");
                                    encoderDrive(TURN_SPEED, -5, 5, 4.0);
                                    encoderDrive(7, 45, 45, 5);
                                    encoderDrive(DRIVE_SPEED, -10, -10, 2);
                                    encoderDrive(TURN_SPEED, 16, -16, 4.0);
                                    encoderDrive(DRIVE_SPEED, -8, -8, 2);
                                    encoderDrive(TURN_SPEED, -6, 6, 1);
                                    encoderDrive(10, -50, -50, 2);
                                    encoderDrive(TURN_SPEED, 5, -5, 1);
                                    //encoderDrive (TURN_SPEED, -5, 5, 3);
                                    //encoderDrive (DRIVE_SPEED, -15, -15 , 3);
                                    //encoderDrive (TURN_SPEED, 5, -5, 3);
                                    //encoderDrive (DRIVE_SPEED, -58, -58 , 3);
                                    robot.flagServo.setPosition(0.1);
                                    sleep(4000);
                                    robot.flagServo.setPosition(0.8);
                                    encoderDrive(TURN_SPEED, -5, 5, 1);
                                    encoderDrive(10, 80, 80, 4);
                                    telemetry.addData("Path", "Complete");
                                    telemetry.update();
                                }
                            } else {
                                telemetry.addData("SilverMineral1X", silverMineral1X);
                                telemetry.addData("GoldMineralX", goldMineralX);
                                if (silverMineral2X > goldMineralX) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    encoderDrive(TURN_SPEED, -5, 5, 4.0);
                                    encoderDrive(DRIVE_SPEED, 6, 6, 3);
                                    encoderDrive(TURN_SPEED, 5, -8, 3);
                                    encoderDrive(DRIVE_SPEED, 25, 25, 3);
                                    encoderDrive(DRIVE_SPEED, -12, -12, 3);
                                    encoderDrive(TURN_SPEED, -13, 13, 3);
                                    encoderDrive(DRIVE_SPEED, 40, 40, 3);
                                    encoderDrive(TURN_SPEED, 20, -20, 3);
                                    encoderDrive(DRIVE_SPEED, -40, -40, 4);
                                    robot.flagServo.setPosition(0.1);
                                    sleep(4000);
                                    robot.flagServo.setPosition(0.8);
                                    encoderDrive(DRIVE_SPEED, 80, 80, 4);
                                    telemetry.addData("Path", "Complete");
                                    telemetry.update();
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Left2");
                                    encoderDrive(TURN_SPEED, -5, 5, 4.0);
                                    encoderDrive(7, 45, 45, 5);
                                    encoderDrive(DRIVE_SPEED, -10, -10, 2);
                                    encoderDrive(TURN_SPEED, 16, -16, 4.0);
                                    encoderDrive(DRIVE_SPEED, -8, -8, 2);
                                    encoderDrive(TURN_SPEED, -6, 6, 1);
                                    encoderDrive(10, -50, -50, 2);
                                    encoderDrive(TURN_SPEED, 5, -5, 1);
                                    //encoderDrive (TURN_SPEED, -5, 5, 3);
                                    //encoderDrive (DRIVE_SPEED, -15, -15 , 3);
                                    //encoderDrive (TURN_SPEED, 5, -5, 3);
                                    //encoderDrive (DRIVE_SPEED, -58, -58 , 3);
                                    robot.flagServo.setPosition(0.1);
                                    sleep(4000);
                                    robot.flagServo.setPosition(0.8);
                                    encoderDrive(TURN_SPEED, -5, 5, 1);
                                    encoderDrive(10, 80, 80, 4);
                                    telemetry.addData("Path", "Complete");
                                    telemetry.update();
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
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
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

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
