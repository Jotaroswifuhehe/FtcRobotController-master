package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "Blah")
public class Blah extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.speak("Plums don't feel right");
         sleep(5000);
    }
}
