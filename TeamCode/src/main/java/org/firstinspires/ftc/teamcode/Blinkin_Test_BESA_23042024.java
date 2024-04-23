package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Blinkin Test", group = "Linear OPMode")
public class Blinkin_Test_BESA_23042024 extends LinearOpMode {

    RevBlinkinLedDriver LedStrip;
    // LED strip
    RevBlinkinLedDriver.BlinkinPattern pattern;
    // Pattern
    DigitalChannel digitalTouch1;
    // Touch sensor 1
    DigitalChannel digitalTouch2;
    // Touch sensor 2
    @Override
    public void runOpMode()  {

        LedStrip = hardwareMap.get(RevBlinkinLedDriver.class, "LedStrip");
        // Communicating with the LedStrip

        digitalTouch1 = hardwareMap.get(DigitalChannel.class, "TouchSensor1");
        digitalTouch1.setMode(DigitalChannel.Mode.INPUT);
        // Communicating with TouchSensor1 and setting the mode to input

        digitalTouch2 = hardwareMap.get(DigitalChannel.class, "TouchSensor2");
        digitalTouch2.setMode(DigitalChannel.Mode.INPUT);
        // Communicating with TouchSensor2 and setting mode to input
    }
}
