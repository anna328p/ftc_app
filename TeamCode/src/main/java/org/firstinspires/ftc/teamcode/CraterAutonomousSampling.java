package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Map;

@Autonomous(name = "Crater Autonomous Sampling", group = "Pushbot")
public class CraterAutonomousSampling extends SamplingAutonomous {

    // todo: write your code here
    @Override
    public void runOpMode() {
        super.runOpMode();
        if (targetPos == -1) {
            encoderDrive(DRIVE_SPEED, -16, -15, 5);
            encoderDrive(TURN_SPEED, -8, 8, 5);
            encoderDrive(DRIVE_SPEED, -43, -43, 5);
            encoderDrive(TURN_SPEED, 10, -10, 5);
            encoderDrive(DRIVE_SPEED, 20, 20, 5);
        }
    }
}