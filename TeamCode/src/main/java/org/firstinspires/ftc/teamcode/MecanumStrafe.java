/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Mecanum Strafe Drive", group = "Testing")
public class MecanumStrafe extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();

    // todo: write your code here
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "ready");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    public void move(double lf, double rf, double lb, double rb) {
        robot.leftFrontMotor.setPower(lf);
        robot.leftBackMotor.setPower(lb);
        robot.rightFrontMotor.setPower(rf);
        robot.rightBackMotor.setPower(rb);
    }

    public void move(double left, double right) {
        robot.leftFrontMotor.setPower(left);
        robot.leftBackMotor.setPower(left);
        robot.rightFrontMotor.setPower(right);
        robot.rightBackMotor.setPower(right);
    }

    @Override
    public void loop() {
        double left_x = -gamepad1.left_stick_x;
        double left_y = gamepad1.left_stick_y;
        double right_x = -gamepad1.right_stick_x;

        left_x = left_x * left_x * left_x;
        left_y = left_y * left_y * left_y;
        right_x = right_x * right_x * right_x;

        double r = Math.hypot(left_x, left_y);
        double robotAngle = Math.atan2(left_y, left_x) - Math.PI / 4;
        final double v1 = r * Math.cos(robotAngle) + right_x;
        final double v2 = r * Math.sin(robotAngle) - right_x;
        final double v3 = r * Math.sin(robotAngle) + right_x;
        final double v4 = r * Math.cos(robotAngle) - right_x;

        move(v1, v2, v3, v4);

        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
