package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Straight" , group = "Otter")
//@Disabled
public class AutoStraightEncoder extends LinearOpMode {

    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    public Servo lLinservo;
    public Servo rLinservo;
    public Servo touchMount;

    @Override
    public void runOpMode() throws InterruptedException {
        // Define and Initialize Motors
        leftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        leftRearMotor = hardwareMap.dcMotor.get("left_rear_drive");
        rightRearMotor = hardwareMap.dcMotor.get("right_rear_drive");
        lLinservo = hardwareMap.servo.get("left_linservo");
        rLinservo = hardwareMap.servo.get("right_linservo");

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //tell the wheels to run based off the encoders

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

        touchMount.setPosition(0.2);
        rLinservo.setPosition(0.2);
        lLinservo.setPosition(0.2);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(500);

        while (opModeIsActive() && (rightFrontMotor.getCurrentPosition() > -7500)) {
            leftFrontMotor.setPower(0.5);
            rightFrontMotor.setPower(0.5);
            leftRearMotor.setPower(0.5);
            rightRearMotor.setPower(0.5);

            telemetry.addData("5", "pos: " + rightFrontMotor.getCurrentPosition());
            telemetry.update();
        }

        //Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

    }
}
