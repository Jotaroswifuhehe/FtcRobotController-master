package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class auto extends LinearOpMode {
    //define your hardware
    DcMotor frontLeft , frontRight , backRight , backLeft ;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        waitForStart();
        // Instructions

        forward();
        sleep(1500);

        turnRight();
        sleep(1500);

        turnLeft();
        sleep(750);

        backwards();
        sleep(2000);

        Stop();
    }
    // Behaviors


    //Forward
    public void forward() {
         frontLeft.setPower(.3);
         frontRight.setPower(-.3);
         backLeft.setPower(.3);
         backRight.setPower(-.3);
    }
    //Backwards
    public void backwards() {
        frontLeft.setPower(-.3);
        frontRight.setPower(-.3);
        backLeft.setPower(.3);
        backRight.setPower(.3);
    }

    //Turn right
    public void turnRight() {
    frontLeft.setPower(.3);
    frontRight.setPower(0);
    backLeft.setPower(.3);
    backRight.setPower(0);
       }

    //Turn left
    public void turnLeft() {
        frontLeft.setPower(0);
        frontRight.setPower(.3);
        backLeft.setPower(0);
        backRight.setPower(.3);
    }
    //Stop motors
        public void Stop () {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

    }




