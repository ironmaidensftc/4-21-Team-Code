package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
public class container extends LinearOpMode {

    public static void drive(DcMotor fL, DcMotor rL,
                             DcMotor fR, DcMotor rR,
                             double lPow, double rPow) {
        fL.setPower(lPow);     rL.setPower(lPow);
        fR.setPower(rPow);     rR.setPower(rPow);
    }

    public void forwards(DcMotor fL, DcMotor rL,
                         DcMotor fR, DcMotor rR,
                         double lPow, double rPow,
                         int goal){
        while (opModeIsActive() && fL.getCurrentPosition() < goal) {
            fL.setPower(lPow);     rL.setPower(lPow);
            fR.setPower(rPow);     rR.setPower(rPow);

            telemetry.addData("5", "pos: " + fL.getCurrentPosition());
            telemetry.update();
        }
    }

    public void turn(BNO055IMU imu, Orientation a, double goal, DcMotor fL, DcMotor rL,
                     DcMotor fR, DcMotor rR, double lPow, double rPow, container robot){
        if(goal > 0) {
            while (opModeIsActive() && (Math.abs(a.firstAngle) > goal)) {
                a = imu.getAngularOrientation();

                fL.setPower(lPow);
                rL.setPower(lPow);
                fR.setPower(rPow);
                rR.setPower(rPow);
                telemetry.addData("5", "heading: " + Math.abs(angles.firstAngle));
                telemetry.update();
            }
        }

        else if(goal < 0){
            robot.turnNeg(imu, a, goal, (360 + goal), fL, rL, fR, rR, lPow, rPow);
        }
    }

    public void turnNeg(BNO055IMU imu, Orientation a, double goal1, double goal2, DcMotor fL, DcMotor rL,
                      DcMotor fR, DcMotor rR, double lPow, double rPow){
        while (opModeIsActive() && ((Math.abs(a.firstAngle) > goal2) || (Math.abs(a.firstAngle) < goal1))) {
            a = imu.getAngularOrientation();

            fL.setPower(lPow);     rL.setPower(lPow);
            fR.setPower(rPow);     rR.setPower(rPow);
            telemetry.addData("5", "heading: " + Math.abs(angles.firstAngle));
            telemetry.update();
        }
    }

    public void backwards(DcMotor m, DcMotor fL, DcMotor rL, DcMotor fR, DcMotor rR,
                          double lPow, double rPow, int goal){
        while (opModeIsActive() && m.getCurrentPosition() < goal) {
            fL.setPower(lPow);     rL.setPower(lPow);
            fR.setPower(rPow);     rR.setPower(rPow);

            telemetry.addData("5", "pos: " + m.getCurrentPosition());
            telemetry.update();
        }
    }

    public void odsRun(OpticalDistanceSensor ods, DcMotor fL, DcMotor rL,
                       DcMotor fR, DcMotor rR, double lPow, double rPow, double goal){
        while (opModeIsActive() && ods.getLightDetected() < goal) {
            fL.setPower(lPow);     rL.setPower(lPow);
            fR.setPower(rPow);     rR.setPower(rPow);

            telemetry.addData("5", "left ods: " + lods.getLightDetected());
            telemetry.update();
        }
    }

    public void allStop(DcMotor fL, DcMotor rL, DcMotor fR, DcMotor rR){
        fL.setPower(0);
        rL.setPower(0);
        fR.setPower(0);
        rR.setPower(0);
    }

    public void color(byte[] array, Servo r, Servo l,
                      I2cDeviceSynch read, String color){
        array = read.read(0x04, 1);

        if(color.startsWith("r")){
            if((array[0] >= 10)) {
                r.setPosition(0.8);

                sleep(850);
                l.setPosition(0.2);
                r.setPosition(0.2);
            }
            else {
                l.setPosition(0.8);

                sleep(850);
                l.setPosition(0.2);
                r.setPosition(0.2);
            }
        }
        else if(color.startsWith("b")){
            if((array[0] >= 10)) {
                l.setPosition(0.8);

                sleep(850);
                l.setPosition(0.2);
                r.setPosition(0.2);
            }
            else {
                r.setPosition(0.8);

                sleep(850);
                l.setPosition(0.2);
                r.setPosition(0.2);
            }
        }
    }

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
    }
}