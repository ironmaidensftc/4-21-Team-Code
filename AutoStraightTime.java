package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Straight Time")
@Disabled
public class AutoStraightTime extends LinearOpMode {

    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();

        HardwareMap hwMap =  null;

            // Define and Initialize Motors
            leftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
            rightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
            leftRearMotor = hardwareMap.dcMotor.get("left_rear_drive");
            rightRearMotor = hardwareMap.dcMotor.get("right_rear_drive");

            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            rightRearMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

            // Set all motors to zero power
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

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
