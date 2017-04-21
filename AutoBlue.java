package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import static org.firstinspires.ftc.teamcode.container.*;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Blue One", group = "Blue")
//@Disabled
public class AutoBlue extends LinearOpMode {
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    public Servo lLinservo;
    public Servo rLinservo;
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
        container robot = new container();

        double FORWARD_SPEED = 0.5;
        double TURN_SPEED = 0.5;

        // Define and Initialize Motors
        leftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        leftRearMotor = hardwareMap.dcMotor.get("left_rear_drive");
        rightRearMotor = hardwareMap.dcMotor.get("right_rear_drive");
        lLinservo = hardwareMap.servo.get("left_linservo");
        rLinservo = hardwareMap.servo.get("right_linservo");
        touchMount = hardwareMap.servo.get("touchMount");
        color = hardwareMap.i2cDevice.get("color");
        rods = hardwareMap.opticalDistanceSensor.get("right_ODS");
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
        idle();

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //tell the wheels to run based off the encoders
        idle();

        // Set all motors to zero power
        robot.allStop(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

        touchMount.setPosition(0.3);
        rLinservo.setPosition(0.3);
        lLinservo.setPosition(0.3);


























        //new comment
        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        telemetry.addData("5", "left encoder: " + leftFrontMotor.getCurrentPosition());
        telemetry.update();

        //going forward a little bit
        robot.forwards(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, 0.8, 0.3, 2850);

        // Set all motors to zero power
        robot.allStop(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

        sleep(250);

        angles = imu.getAngularOrientation();
        robot.turnNeg(imu, angles, 30, 320, leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, 0.5, -0.5);

        // Set all motors to zero power
        robot.allStop(leftFrontMotor, leftRearMotor,
                rightFrontMotor, rightRearMotor);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //tell the wheels to run based off the encoders

        sleep(500);

        robot.odsRun(rods,leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, 0.4, 0.4,  0.1);

        // Set all motors to zero power
        robot.allStop(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

        sleep(250);

        int x = leftFrontMotor.getCurrentPosition() - 150;

        robot.backwards(leftFrontMotor, leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, -0.4, -0.4, x);

        // Set all motors to zero power
        robot.allStop(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

        sleep(750);

        angles = imu.getAngularOrientation();

        robot.turn(imu, angles, 74, leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, 0.5, -0.5, robot);

        robot.allStop(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

        //Drive forward
        robot.drive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, 0.1, 0.1);

        //going forward a little bit
        while (opModeIsActive() && (!touch.isPressed())) {
            idle();
        }

        // Set all motors to zero power
        robot.allStop(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

        touchMount.setPosition(0.3);

        sleep(100);

        robot.color(colorCcache, rLinservo, lLinservo, colorCreader, "b");

        sleep(500);

    }
}
