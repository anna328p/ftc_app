package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Crater Autonomous", group = "Pushbot")

public class CraterAutonomous extends EncoderAutonomous {

    // todo: write your code here
    @Override
    public void runOpMode() {
        super.runOpMode();
        encoderDrive(DRIVE_SPEED, 6, 6, 5);
        encoderDrive(TURN_SPEED, 8, -8, 5);
        encoderDrive(DRIVE_SPEED, 45, 45, 5);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}