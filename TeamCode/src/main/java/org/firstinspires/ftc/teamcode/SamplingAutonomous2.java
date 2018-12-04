package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Sampling Trash code", group = "Pushbot")

public class SamplingAutonomous2 extends EncoderAutonomous {

    private int goldAlignment = 0; //-1 = left, 0 = center, 1 = right

    // todo: write your code here
    @Override
    public void runOpMode() {
        super.runOpMode();
        encoderDrive(DRIVE_SPEED, 6, 6, 5);
        if (goldAlignment < 0) {
            //	encoderDrive(TURN_SPEED, -1, 1, 5);
            encoderDrive(0.9, 40, 40, 5);
            encoderDrive(TURN_SPEED, 11, -11, 5);
            encoderDrive(0.9, 30, 30, 1);
            encoderDrive(DRIVE_SPEED, -5, -5, 3);
            encoderDrive(TURN_SPEED, 30, -30, 5);
        } else if (goldAlignment == 0) {
            encoderDrive(DRIVE_SPEED, 10, 10, 5);
            encoderDrive(TURN_SPEED, 30, -30, 5);
            encoderDrive(0.9, -35, -35, 5);
            encoderDrive(DRIVE_SPEED, 10, 10, 3);

            //encoderDrive(TURN_SPEED, 25, 25, 5);
        }
        //robot.collectorJoint1.setPower(0.0);
        //encoderDrive(TURN_SPEED,   25, -25, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
		
		/*robot.collectorJoint1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.collectorJoint1.setTargetPosition((int)(1680 * 1.3));
		robot.collectorJoint1.setPower(1.0);
		*/
	/*	robot.dumpServo.setPosition(0.1);
		sleep(4000);
		encoderDrive(TURN_SPEED, -6, 6, 5.0);
		robot.dumpServo.setPosition(0.8);
		encoderDrive(0.9, 72, 72, 10);
		telemetry.addData("Path", "Complete");
		telemetry.update();*/
    }
}
