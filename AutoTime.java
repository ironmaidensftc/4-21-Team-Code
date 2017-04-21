package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Line, Time")
@Disabled
public class AutoTime extends LinearOpMode {

    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;
    public Servo lLinservo;
    public Servo rLinservo;
    public Servo touchMount;
    public I2cDevice color = null;
    public GyroSensor gyro = null;
    I2cDeviceSynch colorReader;
    OpticalDistanceSensor ods = null;
    TouchSensor touch = null;

    byte[] colorCcache;
    I2cDeviceSynch colorCreader;
    boolean LEDState = true;

    @Override
    public void runOpMode() throws InterruptedException {
        double FORWARD_SPEED = 0.75;
        double TURN_SPEED = 0.5;
        gyro.calibrate();
        ElapsedTime runtime = new ElapsedTime();

        colorCreader = new I2cDeviceSynchImpl(color, new I2cAddr(0x1e), false);
        colorCreader.engage();
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        HardwareMap hwMap =  null;

        // Define and Initialize Motors
        leftFrontMotor = hwMap.dcMotor.get("left_drive");
        rightFrontMotor = hwMap.dcMotor.get("right_drive");
        leftRearMotor = hwMap.dcMotor.get("left_drive");
        rightRearMotor = hwMap.dcMotor.get("right_drive");
        lLinservo = hardwareMap.servo.get("left_linservo");
        rLinservo = hardwareMap.servo.get("right_linservo");
        touchMount = hardwareMap.servo.get("touchMount");
        color = hardwareMap.i2cDevice.get("color");
        ods = hardwareMap.opticalDistanceSensor.get("ODS");
        touch = hardwareMap.touchSensor.get("touch");

        colorCreader.write8(3, 0);    //Set the mode of the color sensor using LEDState

        boolean State = true;//Tracks the mode of the color sensor; Active = true, Passive = false

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        touchMount.setPosition(0.7);

        //Drive forward for 1/2 second
        leftFrontMotor.setPower(FORWARD_SPEED);
        rightFrontMotor.setPower(FORWARD_SPEED);
        leftRearMotor.setPower(FORWARD_SPEED);
        rightRearMotor.setPower(FORWARD_SPEED);

        runtime.reset();
        gyro.calibrate();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            idle();
        }

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

        sleep(75);

        gyro.calibrate();

        //Turn for 45 degrees
        leftFrontMotor.setPower(TURN_SPEED);
        rightFrontMotor.setPower(TURN_SPEED);
        leftRearMotor.setPower(TURN_SPEED);
        rightRearMotor.setPower(TURN_SPEED);

        runtime.reset();
        while (opModeIsActive() && (gyro.getHeading() < -46)) {
            idle();
        }

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

        //Drive forward for 1.74 seconds
        leftFrontMotor.setPower(FORWARD_SPEED);
        rightFrontMotor.setPower(FORWARD_SPEED);
        leftRearMotor.setPower(FORWARD_SPEED);
        rightRearMotor.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.75)) {
            idle();
        }

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

        sleep(75);

        //Drive forward until you hit the white line
        leftFrontMotor.setPower(FORWARD_SPEED);
        rightFrontMotor.setPower(FORWARD_SPEED);
        leftRearMotor.setPower(FORWARD_SPEED);
        rightRearMotor.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (ods.getLightDetected() < 0.25)) {
            idle();
        }

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

        sleep(75);
        runtime.reset();

        //follow the white line for 3 seconds
        while (opModeIsActive() && (ods.getLightDetected() < 0.25) && (!touch.isPressed())) {
            if (ods.getLightDetected() > 0.25) {
                leftFrontMotor.setPower(0.3);
                rightFrontMotor.setPower(0.1);
                leftRearMotor.setPower(0.3);
                rightRearMotor.setPower(0.1);
        } else {
                leftFrontMotor.setPower(0.1);
                rightFrontMotor.setPower(0.3);
                leftRearMotor.setPower(0.1);
                rightRearMotor.setPower(0.3);
            }
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

        touchMount.setPosition(0.3);

        colorCcache = colorCreader.read(0x04, 1);
        if((colorCcache[0] >= 10)) {
            rLinservo.setPosition(0.7);
        }
        else {
            lLinservo.setPosition(0.7);
        }

        sleep(500);
        lLinservo.setPosition(0.3);
        rLinservo.setPosition(0.3);
    }
}
