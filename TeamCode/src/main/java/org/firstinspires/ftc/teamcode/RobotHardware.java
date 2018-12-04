package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by naluz on 9/29/2017.
 */

public class RobotHardware {
    public DcMotor leftFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightBackMotor = null;

    public DcMotor dumpMotor = null;
    public Servo dumpJoint1 = null;
    public Servo dumpJoint2 = null;

    public DcMotor liftMotor = null;

    public DcMotor collectorMotor = null;
    public Servo collectorJoint1 = null;
    public DcMotor collectorLifter = null;
    public Servo collectorBackClaw = null;
    public Servo collectorFrontClaw = null;

    public Servo flagServo = null;

    public BNO055IMU imu = null;

    HardwareMap hwMap = null;

    public RobotHardware() {

    }

    public void init(HardwareMap ahwMap) {
        // save reference to hardware map
        hwMap = ahwMap;

        leftFrontMotor = hwMap.get(DcMotor.class, "lf motor");
        leftBackMotor = hwMap.get(DcMotor.class, "lb motor");
        rightFrontMotor = hwMap.get(DcMotor.class, "rf motor");
        rightBackMotor = hwMap.get(DcMotor.class, "rb motor");

        dumpMotor = hwMap.get(DcMotor.class, "dump motor");
        dumpJoint1 = hwMap.get(Servo.class, "dump joint 1");
        dumpJoint2 = hwMap.get(Servo.class, "dump joint 2");

        liftMotor = hwMap.get(DcMotor.class, "lift motor");

        collectorMotor = hwMap.get(DcMotor.class, "collector motor");
        collectorJoint1 = hwMap.get(Servo.class, "collector joint 1");
        collectorLifter = hwMap.get(DcMotor.class, "collector lifter");
        collectorBackClaw = hwMap.get(Servo.class, "collector back claw");
        collectorFrontClaw = hwMap.get(Servo.class, "collector front claw");

        flagServo = hwMap.get(Servo.class, "flag servo");

        imu = hwMap.get(BNO055IMU.class, "imu");

        // set motor directions

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorMotor.setDirection(DcMotor.Direction.REVERSE);

        //set all motors to zero power

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        liftMotor.setPower(0);

        collectorMotor.setPower(0);
        collectorLifter.setPower(0.0);
        collectorLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dumpMotor.setPower(0.0);
        //flagServo.setPosition(0);

        // IMU initialization

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }
}
