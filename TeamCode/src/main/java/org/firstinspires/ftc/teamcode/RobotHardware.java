package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by naluz on 9/29/2017.
 */

public class RobotHardware {
    public DcMotor leftFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightBackMotor = null;

    public BNO055IMU imu = null;

    HardwareMap hwMap = null;

    public RobotHardware() {

    }

    public void init(HardwareMap ahwMap) {
        // save reference to hardware map
        hwMap = ahwMap;

        leftFrontMotor = hwMap.get(DcMotor.class, "lf");
        leftBackMotor = hwMap.get(DcMotor.class, "lb");
        rightFrontMotor = hwMap.get(DcMotor.class, "rf");
        rightBackMotor = hwMap.get(DcMotor.class, "rb");

        //imu = hwMap.get(BNO055IMU.class, "imu");

        // set motor directions

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        //set all motors to zero power

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        // IMU initialization

        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // parameters.mode = BNO055IMU.SensorMode.IMU;
        //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.loggingEnabled = true;
        //parameters.loggingTag = "IMU";

        //imu.initialize(parameters);
    }
}
