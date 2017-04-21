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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="REDShooterTestNew", group = "Otter")
//@Disabled
public class REDShooterTestNew extends LinearOpMode {

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
        idle();

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //tell the wheels to run based off the encoders
        idle();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("5", "left encoder: " + leftFrontMotor.getCurrentPosition());
        telemetry.update();

        while (opModeIsActive()) {

            // Set all motors to zero power
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

            // Set shooter motors to 50 percent power
            lShooter.setPower(-0.5);
            rShooter.setPower(0.5);

            // Set convader belt to 100 percent power
            conveyor.setPower(-1);

            touchMount.setPosition(0.2);
            rLinservo.setPosition(0.2);
            lLinservo.setPosition(0.2);

            sleep(750);

            //set all motors to zero power
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);
            lShooter.setPower(-0.5);
            rShooter.setPower(0.5);
            conveyor.setPower(-1);

            telemetry.addData("5", "left encoder: " + leftFrontMotor.getCurrentPosition());
            telemetry.update();

            //Drive forward
            leftFrontMotor.setPower(0.6);
            rightFrontMotor.setPower(0.3);
            leftRearMotor.setPower(0.6);
            rightRearMotor.setPower(0.3);

            //going forward a little bit
            while (opModeIsActive() && (leftFrontMotor.getCurrentPosition() < 1500)) {
                idle();

                telemetry.addData("5", "left front: " + leftFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Set all motors to zero power
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

            sleep(250);

            //Turn for 45 degrees
            leftFrontMotor.setPower(-TURN_SPEED);
            rightFrontMotor.setPower(TURN_SPEED);
            leftRearMotor.setPower(-TURN_SPEED);
            rightRearMotor.setPower(TURN_SPEED);

            angles = imu.getAngularOrientation();

            while (opModeIsActive() && ((Math.abs(angles.firstAngle) > 330) ||
                    (Math.abs(angles.firstAngle) == 0))) {
                angles = imu.getAngularOrientation();

                idle();
                telemetry.addData("5", "heading: " + Math.abs(angles.firstAngle));
                telemetry.update();
            }

            // Set all motors to zero power
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //tell the wheels to run based off the encoders

            sleep(500);

            //Drive forward
            leftFrontMotor.setPower(0.6);
            rightFrontMotor.setPower(0.6);
            leftRearMotor.setPower(0.6);
            rightRearMotor.setPower(0.6);

            while (opModeIsActive() && lods.getLightDetected() < 0.05) {
                idle();

                telemetry.addData("5", "right ods: " + rods.getLightDetected());
                telemetry.addData("6", "left front: " + leftFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Set all motors to zero power
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

            telemetry.addData("5", "left encoder: " + leftFrontMotor.getCurrentPosition());
            telemetry.update();

            sleep(250);

            int x = leftFrontMotor.getCurrentPosition() - 260;

            while(opModeIsActive() && leftFrontMotor.getCurrentPosition() > x){
                leftFrontMotor.setPower(-0.4);
                rightFrontMotor.setPower(-0.4);
                leftRearMotor.setPower(-0.4);
                rightRearMotor.setPower(-0.4);

                telemetry.addData("5", "left encoder: " + leftFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Set all motors to zero power
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

            sleep(750);

            //Turn for 45 degrees
            leftFrontMotor.setPower(-TURN_SPEED);
            rightFrontMotor.setPower(TURN_SPEED);
            leftRearMotor.setPower(-TURN_SPEED);
            rightRearMotor.setPower(TURN_SPEED);

            angles = imu.getAngularOrientation();

            while (opModeIsActive() &&
                    (Math.abs(angles.firstAngle) > 285)) {
                angles = imu.getAngularOrientation();

                idle();
                telemetry.addData("5", "heading: " + Math.abs(angles.firstAngle));
                telemetry.update();
            }

            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

            //Drive forward
            leftFrontMotor.setPower(0.2);
            rightFrontMotor.setPower(0.2);
            leftRearMotor.setPower(0.2);
            rightRearMotor.setPower(0.2);

            //going forward a little bit
            while (opModeIsActive() && (!touch.isPressed())) {
                idle();
            }

            // Set all motors to zero power
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

            touchMount.setPosition(0.3);

            sleep(100);

            colorCcache = colorCreader.read(0x04, 1);

            if((colorCcache[0] >= 10)) {
                rLinservo.setPosition(0.8);

                sleep(850);
                lLinservo.setPosition(0.2);
                rLinservo.setPosition(0.2);
            }
            else {
                lLinservo.setPosition(0.8);

                sleep(850);
                lLinservo.setPosition(0.2);
                rLinservo.setPosition(0.2);
            }

            sleep(500);

        }
    }
}