package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "ServoTest", group = "Iterative Opmode")

public class ServoTestAutonomous extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double servoPos = 0.0;
    double servoMod = 1.0 / 64.0;

    @Override
    public void loop() {
		/*if (servoPos > 2 || servoPos < 0) {
			servoMod = -servoMod;	
		}
		servoPos += servoMod;
		robot.collectorServo.setPosition(servoPos);*/
        try {
            //robot.collectorServo.setPosition(0.7);
            TimeUnit.MILLISECONDS.sleep(1200);
            //robot.collectorServo.setPosition(4);
            TimeUnit.MILLISECONDS.sleep(1200);
        } catch (InterruptedException e) {
            System.exit(1);
        }

        // Show the elapsed game time and servo positions.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Servo State", "ServoPos " + servoPos + " ServoMod " + servoMod);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
