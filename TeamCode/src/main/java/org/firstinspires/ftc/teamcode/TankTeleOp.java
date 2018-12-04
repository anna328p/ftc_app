package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tank TeleOp", group = "Robot")
public class TankTeleOp extends OpMode {
    RobotHardware robot = new RobotHardware();

    // todo: write your code here
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("status", "ready");
        telemetry.update();
    }

    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        telemetry.addData("input", "forward " + forward + " turn " + turn);
        double leftPower = forward + turn;
        double rightPower = forward - turn;
        robot.leftFrontMotor.setPower(leftPower);
        robot.rightFrontMotor.setPower(rightPower);
        robot.leftBackMotor.setPower(leftPower);
        robot.rightBackMotor.setPower(rightPower);

        if (gamepad1.left_bumper) {
            robot.liftMotor.setPower(1.0);
        } else if (gamepad1.right_bumper) {
            robot.liftMotor.setPower(-1.0);
        } else {
            robot.liftMotor.setPower(0.0);
        }

        if (gamepad1.right_trigger > 0.1) {
            robot.collectorMotor.setPower(1.0);
            telemetry.addData("power", 1.0);
        } else if (gamepad1.left_trigger > 0.1) {
            robot.collectorMotor.setPower(-1.0);
            telemetry.addData("power", -1.0);
        } else {
            robot.collectorMotor.setPower(0.0);
            telemetry.addData("power", 0.0);
        }

        if (gamepad1.dpad_up) {
            robot.collectorLifter.setPower(1.0);
        } else if (gamepad1.dpad_down) {
            robot.collectorLifter.setPower(-1.0);
        } else {
            robot.collectorLifter.setPower(0.0);
        }
        telemetry.addData("collector lifter", robot.collectorLifter.getCurrentPosition());

        if (gamepad1.dpad_right) {
            robot.collectorJoint1.setPosition(0.0);
        } else if (gamepad1.dpad_left) {
            robot.collectorJoint1.setPosition(1.0);
        } else {
            robot.collectorJoint1.setPosition(0.495);
        }

        if (gamepad2.right_trigger > 0.1) {
            robot.dumpMotor.setPower(1.0);
        } else if (gamepad2.left_trigger > 0.1) {
            robot.dumpMotor.setPower(-1.0);
        } else {
            robot.dumpMotor.setPower(0.0);
        }
		
		/*if (gamepad1.a) {
			robot.collectorJoint1.setPosition(1.0);
		} else {
			robot.collectorJoint1.setPosition(0.5);
		}*/
        //robot.collectorJoint1.setPosition(0.5);

        if (gamepad1.y) {
            robot.collectorBackClaw.setPosition(0.9);
        } else {
            robot.collectorBackClaw.setPosition(0.1);
        }

        if (gamepad1.b) {
            robot.collectorFrontClaw.setPosition(0.3);
        } else if (gamepad1.a) {
            robot.collectorFrontClaw.setPosition(0.9);
        } else {
            robot.collectorFrontClaw.setPosition(0.5);
        }

        if (gamepad2.a) {
            robot.dumpJoint1.setPosition(0.1);
        } else if (gamepad2.y) {
            robot.dumpJoint1.setPosition(0.9);
        } else {
            robot.dumpJoint1.setPosition(0.47);
        }
		
		/*if (gamepad2.b) {
			robot.dumpJoint2.setPosition(0.8);
		} else {
			robot.dumpJoint2.setPosition(0.5);
		}*/
        robot.dumpJoint2.setPosition(0.8);
        telemetry.update();
    }
}
