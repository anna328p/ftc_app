package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Reset Lifter", group = "Pushbot")

public class ResetLifter extends LinearOpMode {

    // todo: write your code here
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setTargetPosition(0);
        robot.liftMotor.setPower(1.0);
        while (robot.liftMotor.isBusy()) {
            telemetry.addData("lifter", robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}