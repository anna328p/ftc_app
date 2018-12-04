package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Crater Flag Autonomous", group = "Pushbot")

public class CraterFlagAutonomous extends EncoderAutonomous {

    // todo: write your code here
    @Override
    public void runOpMode() {
        super.runOpMode();
        encoderDrive(DRIVE_SPEED, 6, 6, 5);
        encoderDrive(TURN_SPEED, 6, -6, 5);
        encoderDrive(0.9, 16, 16, 5);

        //encoderDrive(TURN_SPEED, 8, -8, 4.0);
        //encoderDrive(1.0, -60, -60, 5.0);
        encoderDrive(TURN_SPEED, 10, -10, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(1.0, -24, -24, 5.0);
        encoderDrive(TURN_SPEED, -2, 2, 5.0);
        encoderDrive(1.0, -36, -36, 5.0);
		/*robot.collectorJoint1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.collectorJoint1.setTargetPosition((int)(1680 * 1.3));
		robot.collectorJoint1.setPower(1.0);
		*/
        robot.flagServo.setPosition(0.1);
        sleep(4000);
        robot.flagServo.setPosition(0.8);
        encoderDrive(1.0, 70, 70, 5.0);
        //encoderDrive(0.9, 72, 72, 10);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}