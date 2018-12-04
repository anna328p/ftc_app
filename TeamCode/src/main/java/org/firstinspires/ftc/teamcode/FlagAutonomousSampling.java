package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Map;

@Autonomous(name = "Flag Autonomous Sampling", group = "Pushbot")
public class FlagAutonomousSampling extends SamplingAutonomous {

    // todo: write your code here
    @Override
    public void runOpMode() {
        super.runOpMode();
        if (targetPos == 1) {
            encoderDrive(TURN_SPEED, -15, 15, 5);
            encoderDrive(DRIVE_SPEED, 27, 27, 5);
            encoderDrive(TURN_SPEED, 20, -20, 5);
        } else if (targetPos == 0) {
            encoderDrive(DRIVE_SPEED, 20, 20, 5);
            encoderDrive(TURN_SPEED, 11, -11, 5);
            //encoderDrive (DRIVE_SPEED, 23, 22, 5);
            //encoderDrive (TURN_SPEED, 13, -13, 5);
        } else if (targetPos == -1) {
            encoderDrive(DRIVE_SPEED, 8, 8, 5);
            encoderDrive(TURN_SPEED, 11, -11, 5);
            encoderDrive(DRIVE_SPEED, 35, 35, 5);
            encoderDrive(TURN_SPEED, 3, -3, 5);
        }
        robot.flagServo.setPosition(0.2);
        sleep(1500);
        robot.flagServo.setPosition(0.8);
        if (targetPos == 1) {
			/*encoderDrive (DRIVE_SPEED, 5, 5, 5); //drive forward into team's crater
			encoderDrive (TURN_SPEED, 8, -8, 5);
			encoderDrive (1.0, 66, 66, 6);*/

            encoderDrive(1.0, -21, -21, 5);
            encoderDrive(TURN_SPEED, -4, 4, 5);
            encoderDrive(DRIVE_SPEED, -56, -56, 6);
        } else if (targetPos == 0) {
            encoderDrive(DRIVE_SPEED, -22, -22, 6);
            encoderDrive(TURN_SPEED, -4, 4, 5);
            encoderDrive(1.0, -51, -51, 5);
        } else if (targetPos == -1) {
            encoderDrive(1.0, -73, -73, 5);
        }
    }
}