package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Flag Autonomous", group = "Pushbot")

public class FlagAutonomous extends EncoderAutonomous {

    // todo: write your code here
    @Override
    public void runOpMode() {
        super.runOpMode();
        encoderDrive(DRIVE_SPEED, 6, 6, 5);
        encoderDrive(TURN_SPEED, 6, -6, 5);
        encoderDrive(0.9, 48, 48, 5);
        encoderDrive(TURN_SPEED, 25, -25, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
		
		/*robot.collectorJoint1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.collectorJoint1.setTargetPosition((int)(1680 * 1.3));
		robot.collectorJoint1.setPower(1.0);
		*/
        //robot.dumpServo.setPosition(0.1);
        sleep(4000);
        encoderDrive(TURN_SPEED, -6, 6, 5.0);
        //robot.dumpServo.setPosition(0.8);
        encoderDrive(0.9, 72, 72, 10);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}