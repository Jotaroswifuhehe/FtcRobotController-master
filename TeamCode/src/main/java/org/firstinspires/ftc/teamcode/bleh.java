package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "bleh")
@Disabled
public class bleh extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    Servo servoBoi;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        servoBoi = hardwareMap.servo.get("servoBoi");}
    {

            waitForStart();
        //b for 3 seconds
        frontRight.setPower(.12);
        frontLeft.setPower(.12);
        backLeft.setPower(.12);
        backRight.setPower(.12);
        sleep(01);
        //rotate for 2.5 seconds
        sleep(0);

    } }

