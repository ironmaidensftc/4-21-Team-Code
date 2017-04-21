package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**Controller Map - Controller #1
 * Joysticks - Tank Drive, Wheels
 * Y -
 * B - Right Linear Servo, In and Out
 * A -
 * X - Left Linear Servo, In and Out
 * D-Pad Up -
 * D-Pad Right -
 * D-Pad Down -
 * D-Pad Left -
 * Left Bumper -
 * Left Trigger -
 * Right Bumper -
 * Right Trigger - Slow-Mo
 */


/**Controller Map - Controller #2
 * Joysticks -
 * Y -
 * B - Shoop Shoop (Collection Device) - Out
 * A -
 * X - Shoop Shoop (Collection Device) - In
 * D-Pad Up -
 * D-Pad Right -
 * D-Pad Down -
 * D-Pad Left -
 * Left Bumper -
 * Left Trigger - Conveyor Belt Toggle
 * Right Bumper - Shooter Toggle
 * Right Trigger -
 */

@TeleOp(name="Taco Base")
//@Disabled
public class TacoBase extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    public DcMotor conveyor;
    public DcMotor lShooter;
    public DcMotor rShooter;
    public Servo lLinservo;
    public Servo rLinservo;
    public DcMotor shoopShoop;
    public I2cDevice color;
    I2cDeviceSynch colorCreader;

    @Override
    public void runOpMode() throws InterruptedException {
        int leftLinCount = 0;
        int rightLinCount = 0;
        double div = 0.75;
        int conveyorCount = 0;
        int shooterCount = 0;

        // Define and initialize motors, servos, and sensors
        leftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        leftRearMotor = hardwareMap.dcMotor.get("left_rear_drive");
        rightRearMotor = hardwareMap.dcMotor.get("right_rear_drive");
        shoopShoop = hardwareMap.dcMotor.get("shoop_shoop");
        conveyor = hardwareMap.dcMotor.get("conveyor");
        lShooter = hardwareMap.dcMotor.get("lShooter");
        rShooter = hardwareMap.dcMotor.get("rShooter");
        lLinservo = hardwareMap.servo.get("left_linservo");
        rLinservo = hardwareMap.servo.get("right_linservo");
        color = hardwareMap.i2cDevice.get("color");

        colorCreader = new I2cDeviceSynchImpl(color, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        colorCreader.write8(3, 1);

        //set the right motors' directions to reverse
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        shoopShoop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        lShooter.setMode
                (DcMotor.RunMode.RUN_USING_ENCODER);
        rShooter.setMode
                (DcMotor.RunMode.RUN_USING_ENCODER);
        lShooter.setZeroPowerBehavior
                (DcMotor.ZeroPowerBehavior.FLOAT);
        rShooter.setZeroPowerBehavior
                (DcMotor.ZeroPowerBehavior.FLOAT);
        lShooter.setMaxSpeed(60);
        rShooter.setMaxSpeed(60);

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        shoopShoop.setPower(0);
        //Set the linear servos to its base position of 0.3
        lLinservo.setPosition(0.3);
        rLinservo.setPosition(0.3);


        // Wait for the game to start
        waitForStart();

        // run until the end of the match
        while (opModeIsActive()) {
            double lPow;
            double rPow;
            int up = 0;

            //assign the x and y variables to the inputs from the joysticks
            float x = -gamepad1.left_stick_y; //left
            float y = -gamepad1.right_stick_y; //right

            //ensure the x and y values will not go over +-1 (a motors' max and min)
            x = Range.clip(x, -1, 1);
            y = Range.clip(y, -1, 1);

            //run the motors at their respective powers derived from the joysticks
            leftFrontMotor.setPower(x * div);
            rightFrontMotor.setPower(y * div);
            leftRearMotor.setPower(x * div);
            rightRearMotor.setPower(y * div);

            if((gamepad2.left_trigger > 0.35) && conveyorCount == 0){
                conveyor.setPower(-0.5);

                conveyorCount++;

                sleep(750);
            }
            else if (gamepad2.left_trigger > 0.35 && conveyorCount == 1){
                conveyor.setPower(0);

                conveyorCount = 0;

                sleep(750);
            }

            if (gamepad2.right_bumper && shooterCount == 0){
                lShooter.setPower(-1);
                rShooter.setPower(1);

                shooterCount = 1;

                sleep(750);
            }

            else if (gamepad2.right_bumper && shooterCount == 1) {
                lShooter.setPower(0);
                rShooter.setPower(0);

                shooterCount = 0;

                sleep(750);
            }

            //if the x button on the gamepad is pressed and the counter is 0...
            if (gamepad1.x && leftLinCount == 0) {
                //set the left linear servo to the extended position
                lLinservo.setPosition(0.7);
                leftLinCount++;

                sleep(500);
            }

            //if the x button on the gamepad is pressed and the counter is NOT 0...
            else if (gamepad1.x && leftLinCount != 0) {
                //set the left linear servo to the retracted position
                lLinservo.setPosition(0.3);
                leftLinCount = 0;

                sleep(500);
            }

            //if the b button on the gamepad is pressed and the counter is 0...
            if (gamepad1.b && rightLinCount == 0) {
                //set the right linear servo to the extended position
                rLinservo.setPosition(0.7);
                rightLinCount++;

                sleep(500);
            }

            //if the b button on the gamepad is pressed and the counter is NOT 0...
            else if (gamepad1.b && rightLinCount != 0) {
                //set the right linear servo to the retracted position
                rLinservo.setPosition(0.3);
                rightLinCount = 0;

                sleep(500);
            }

            //if the x button on the gamepad is pressed...
            if(gamepad2.x) {
                //run the shoopshoop (collection device)
                //to be held down in order to run
                shoopShoop.setPower(1);
            }
            //if the b button on the gamepad is pressed...
            else if(gamepad2.b) {
                //run the shoopshoop backwards (collection device)
                //to be held down in order to run
                shoopShoop.setPower(-1);
            }
            else {
                shoopShoop.setPower(0);
            }

            if(gamepad1.right_trigger > 0.35) {
                div = 0.2;
            }
            else{
                div = 0.75;
            }

        }

    }

}


/*double powL = lShooter.getPower();
                double powR = rShooter.getPower();

                while(opModeIsActive() && (lShooter.getPower() >= 0.1)
                        || (rShooter.getPower() >= 0.1)){
                    powL -= 0.05;
                    powR -= 0.05;

                    lShooter.setPower(powL);
                    rShooter.setPower(powR);
                    sleep(100);
                }

                lShooter.setPower(0);
                rShooter.setPower(0);

                shooterCount = 0;

                sleep(750);*/