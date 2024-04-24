package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Blinkin Test", group = "Linear OPMode")
public class Blinkin_Test_BESA_23042024 extends LinearOpMode {

    RevBlinkinLedDriver ledStrip;
    // LED strip
    RevBlinkinLedDriver.BlinkinPattern pattern;
    // Pattern
    DigitalChannel digitalTouch1;
    // Touch sensor 1
    DigitalChannel digitalTouch2;
    // Touch sensor 2
    boolean variableForTouch1 = false;
    boolean variableForTouch2 = false;
    float patternNumber = 0;

    @Override
    public void runOpMode()  {

        ledStrip = hardwareMap.get(RevBlinkinLedDriver.class, "LedStrip");
        pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
        ledStrip.setPattern(pattern);
        // Communicating with the LedStrip and setting idle to STROBE_GOLD

        digitalTouch1 = hardwareMap.get(DigitalChannel.class, "TouchSensor1");
        digitalTouch1.setMode(DigitalChannel.Mode.INPUT);
        // Communicating with TouchSensor1 and setting the mode to input

        digitalTouch2 = hardwareMap.get(DigitalChannel.class, "TouchSensor2");
        digitalTouch2.setMode(DigitalChannel.Mode.INPUT);
        // Communicating with TouchSensor2 and setting mode to input

        telemetry.addData("Blinkin LED controller Test Program", "Press Start to Initiate Test.");
        telemetry.update();

        waitForStart();
        // Wait for starting condition

        while (opModeIsActive()) {
        // When play is pressed, run the code inside this loop
            if (digitalTouch1.getState()) {
                if (!variableForTouch1) {
                    patternNumber++;
                    if (patternNumber > 17) {
                        patternNumber = 18;
                    }
                    variableForTouch1 = true;
                }
            } else {
                variableForTouch1 = false;
            }
            // incrementing pattern number once per button press, wrapping the the bottom when finished
            // if you hold button down it will not increment any more
            if (digitalTouch2.getState()) {
                if (!variableForTouch2) {
                    patternNumber--;
                    if (patternNumber < 0) {
                        patternNumber = 17;
                    }
                    variableForTouch2 = true;
                }
            } else {
                variableForTouch2 = false;
            }
            // decrementing pattern number once per button press, wrapping to the top when finished
            // if you hold the button down it will not decrement any more

        }
    }
}
