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
    // Allows us to limit the incrementing to once per button press, see below for code
    boolean variableForTouch2 = false;
    // Allows us to limit the decrementing to once per button press, see below for code
    int patternNumber = 0;
    // Allows us to assign a number to a pattern, thus allowing us to increment and decrement through a list, see below for code

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
            // Incrementing pattern number once per button press, wrapping the the bottom when finished
            // If you hold button down it will not increment more than once

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
            // Decrementing pattern number once per button press, wrapping to the top when finished
            // If you hold the button down it will not decrement more than once

            // There are 17 cases, you can expand or hide the code to the right of the line number
            switch (patternNumber) {
                case (0):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
                    break;
                case (1):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
                    break;
                case (2):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    break;
                case (3):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
                    break;
                case (4):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                    break;
                case (5):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
                    break;
                case (6):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    break;
                case (7):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;
                    break;
                case (8):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.LIME;
                    break;
                case (9):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
                    break;
                case (10):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    break;
                case (11):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN;
                    break;
                case (12):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
                    break;
                case (13):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE;
                    break;
                case (14):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                    break;
                case (15):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                    break;
                case (16):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
                    break;
                case (17):
                    pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                    break;
            }
            // Setting the pattern type dependant on the number assigned above

            ledStrip.setPattern(pattern);
            // Setting the led strip to the pattern

            telemetry.addData("Pattern Name: ", pattern.toString());
            telemetry.addData("Pattern Number: ", patternNumber);
            telemetry.addData("Is TouchSensor1 Pressed?: ", digitalTouch1.getState());
            telemetry.addData("variableForTouch1 State: ", variableForTouch1);
            telemetry.addData("Is TouchSensor2 pressed?: ", digitalTouch2.getState());
            telemetry.addData("variableForTouch2 State: ", variableForTouch2);
            telemetry.update();
        }
    }
}
