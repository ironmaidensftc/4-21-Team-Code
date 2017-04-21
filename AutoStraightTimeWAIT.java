package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Auto Straight WAIT" , group = "Otter")
//@Disabled
public class AutoStraightTimeWAIT extends LinearOpMode {

    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    public Servo lLinservo;
    public Servo rLinservo;
    public DcMotor conveyor;
    public DcMotor lShooter;
    public DcMotor rShooter;
    public Servo touchMount;
    public I2cDevice color;
    public BNO055IMU imu;
    public Orientation angles;
    I2cDeviceSynch colorCreader;
    OpticalDistanceSensor rods;
    OpticalDistanceSensor lods;
    TouchSensor touch;

    byte[] colorCcache;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();

        double FORWARD_SPEED = 0.5;
        double TURN_SPEED = 0.5;
        int leftLinCount = 0;
        int rightLinCount = 0;
        double div = 0.75;
        int conveyorCount = 0;
        int shooterCount = 0;

        // Define and Initialize Motors
        leftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        leftRearMotor = hardwareMap.dcMotor.get("left_rear_drive");
        rightRearMotor = hardwareMap.dcMotor.get("right_rear_drive");
        lLinservo = hardwareMap.servo.get("left_linservo");
        rLinservo = hardwareMap.servo.get("right_linservo");
        touchMount = hardwareMap.servo.get("touchMount");
        color = hardwareMap.i2cDevice.get("color");
        conveyor = hardwareMap.dcMotor.get("conveyor");
        lShooter = hardwareMap.dcMotor.get("lShooter");
        rShooter = hardwareMap.dcMotor.get("rShooter");
        rods = hardwareMap.opticalDistanceSensor.get("right_ODS");
        //transfer = hardwareMap.crservo.get("transfer");
        lods = hardwareMap.opticalDistanceSensor.get("left_ODS");
        touch = hardwareMap.touchSensor.get("touch");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        colorCreader = new I2cDeviceSynchImpl(color, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        colorCreader.write8(3, 1);

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //tell the wheels to run based off the encoders
        idle();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(15000);

        //Drive forward for 3 seconds
        leftFrontMotor.setPower(0.75);
        rightFrontMotor.setPower(0.75);
        leftRearMotor.setPower(0.75);
        rightRearMotor.setPower(0.75);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            idle();
        }

        //Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

    }
}
/*while (opModeIsActive() && (leftFrontMotor.getCurrentPosition() > 7500)) {
            leftFrontMotor.setPower(0.5);
            rightFrontMotor.setPower(0.5);
            leftRearMotor.setPower(0.5);
            rightRearMotor.setPower(0.5);

            telemetry.addData("5", "pos: " + leftFrontMotor.getCurrentPosition());
            telemetry.update();
        }*/