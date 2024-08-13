/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Omni Drive By Gyro", group="Robot")
//@Disabled
public class RobotAutoOmniDriveByGyro_SCB extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFront   = null;
    private DcMotor rightFront  = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    // Declare wheel positions so that encoders keep track of robot position.
    int LFPos   = 0;
    int RFPos   = 0;
    int LRPos   = 0;
    int RRPos   = 0;

    private IMU             imu         = null;      // Control/Expansion Hub IMU

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  headingError  = 0;
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  strafeSpeed   = 0;
    private double  turnSpeed     = 0;


    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     STRAFE_SPEED            = 0.3;     // Max strafing speed for drifting accuracy
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     AVG_SPEED               = 0.375;   // Speed when fast is too much and slow is too little.
    static final double     HEADING_THRESHOLD       = 0.25 ;   // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.01;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    // Declare the number of CLICKS_PER_INCH required for the robot to travel a distance of 1 inch
    static double           CLICKS_PER_INCH;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "LeftBack");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");

        // call for drive motor initialization
        initDrvMotors();

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));



             // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        waitForStart();
        // Set the encoders for closed loop speed control, and reset the heading.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        if (opModeIsActive()) {
            // Step through each leg of the path,
            // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
            //          holdHeading() is used after turns to let the heading stabilize
            //          Add a sleep(2000) after any step to keep the telemetry data visible for review

            driveStraight("Forward", DRIVE_SPEED, 8.0, 0.0);    // Drive Forward 8" while holding heading
            driveStraight("Backwards", DRIVE_SPEED, 4.0, 0.0);    // Drive Forward 4" while holding heading

            turnToHeading(TURN_SPEED, 45.0);               // Turn CCW to 45 Degrees
            holdHeading(TURN_SPEED, 45.0, 0.5);   // Hold 45 Deg heading for a 1/2 second

            turnToHeading(TURN_SPEED, -90.0);               // Turn CW to 90 Degrees from start
            holdHeading(TURN_SPEED,-90,0.5);        // Hold 90 Deg heading for 1/2 second

            strafeRobot("StrafeRight", STRAFE_SPEED, 10,-90);

            turnToHeading(TURN_SPEED, 0.0);               // Turn  CCW  to  45 Degrees
            holdHeading(TURN_SPEED, 0.0, 0.5);    // Hold  45 Deg heading for a 1/2 second

            strafeRobot("StrafeLeft", STRAFE_SPEED, 10, 0);

            turnToHeading(TURN_SPEED, 90.0);               // Turn  CW  to 0 Degrees
            holdHeading(TURN_SPEED, 90.0, 1.0);    // Hold  0 Deg heading for 1 second

            driveStraight("Forward",AVG_SPEED, 6.0, 90.0);    // Drive forward 6"

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.
        }
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
    *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the OpMode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    public void driveStraight(String direction, double maxDriveSpeed, double distance, double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Get drive motors current position
            LFPos = leftFront.getCurrentPosition();
            RFPos = rightFront.getCurrentPosition();
            LRPos = leftBack.getCurrentPosition();
            RRPos = rightBack.getCurrentPosition();
            // Determine new target position based on direction given
            if (direction.equals("Forward")) {
                LFPos = (int) (LFPos + distance * CLICKS_PER_INCH);
                RFPos = (int) (RFPos + distance * CLICKS_PER_INCH);
                LRPos = (int) (LRPos + distance * CLICKS_PER_INCH);
                RRPos = (int) (RRPos + distance * CLICKS_PER_INCH);
            }
            else if (direction.equals("Backwards")) {
                LFPos = (int) (LFPos - distance * CLICKS_PER_INCH);
                RFPos = (int) (RFPos - distance * CLICKS_PER_INCH);
                LRPos = (int) (LRPos - distance * CLICKS_PER_INCH);
                RRPos = (int) (RRPos - distance * CLICKS_PER_INCH);
            }

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFront.setTargetPosition(LFPos);
            rightFront.setTargetPosition(RFPos);
            leftBack.setTargetPosition(LRPos);
            rightBack.setTargetPosition(RRPos);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            // Start the drive motors, then initiate moving to position
            moveRobot(maxDriveSpeed, 0, 0);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                   (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (direction.equals("Backwards")) {
                    turnSpeed *= -1.0;
                }

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, 0, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry("Straight");
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            driveMotorsOff();
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Drive sideways (strafe) in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param direction StrafeRight or StrafeLeft, move right or left
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void strafeRobot(String direction, double maxDriveSpeed, double distance, double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Get drive motors current position
            LFPos = leftFront.getCurrentPosition();
            RFPos = rightFront.getCurrentPosition();
            LRPos = leftBack.getCurrentPosition();
            RRPos = rightBack.getCurrentPosition();
            // Determine new target position based on direction given
            if (direction.equals("StrafeRight")) {
                LFPos = (int) (LFPos + distance * CLICKS_PER_INCH);
                RFPos = (int) (RFPos - distance * CLICKS_PER_INCH);
                LRPos = (int) (LRPos - distance * CLICKS_PER_INCH);
                RRPos = (int) (RRPos + distance * CLICKS_PER_INCH);
            } else if (direction.equals("StrafeLeft")) {
                LFPos = (int) (LFPos - distance * CLICKS_PER_INCH);
                RFPos = (int) (RFPos + distance * CLICKS_PER_INCH);
                LRPos = (int) (LRPos + distance * CLICKS_PER_INCH);
                RRPos = (int) (RRPos - distance * CLICKS_PER_INCH);
            }
            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFront.setTargetPosition(LFPos);
            rightFront.setTargetPosition(RFPos);
            leftBack.setTargetPosition(LRPos);
            rightBack.setTargetPosition(RRPos);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            // Start the drive motors, then initiate moving to position
            moveRobot(0, maxDriveSpeed, 0);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if strafing right, the motor correction also needs to be reversed
                if (direction.equals("StrafeRight")) {
                    turnSpeed *= -1.0;
                }

                // Apply the turning correction to the current driving speed.
                moveRobot(0, strafeSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry("Strafe");
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            driveMotorsOff();
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_TURN_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0,0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry("Turning");
        }

        // Stop all motion;
        driveMotorsOff();
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, 0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry("Turning");
        }

        // Stop all motion;
        driveMotorsOff();
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Initialize the drive motors and set motors for direction and operation
     */
    private void initDrvMotors(){
        telemetry.addData("Initializing", "motors and servos...");
        telemetry.update();

        /*
         To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
         When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
         Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        */
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Place motors into braking mode so motors stop abruptly with zero power
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Get the clicks per inch (total revolutions of motor to move 1 inch)
        clicksPerInch();
    }

    /**
     * We need to find the encoder ticks per inch of travel for the given motor and gear ratio
     * Calculate the CLICKS_PER_INCH for your specific drive train.
     *      This will convert distance in inches to encoder ticks.
     * Determine the HD Hex bare motor TICKS_PER_MOTOR_REVOLUTION.
     *      The REV HD Hex Planetary bare motor has 28 ticks per revolution.
     * For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     *      Using external gearing at 15:1 results in 28 x 15 = 420 motor ticks per output shaft revolution.
     * Use 75mm as the diameter of the installed mecanum wheels, converted to inches is 2.953.
     * Calculate wheel circumference in inches: 2.953 x Pi (3.1416) = 9.277 inches
     * This tells u there are 420 motor revolution for every 9.277 inches traveled.
     * If you take 420 divided by 9.277, you get 45.27 ticks per inch of travel.
     */
    private void clicksPerInch() {

        int TICKS_PER_MOTOR_REV = 28;   // REV HD Hex Planetary bare motor
        double DRIVE_GEAR_REDUCTION = 15.0;     // 15:1 Planetary Gearing
        double WHEEL_DIAMETER_INCHES = 2.953;     // (75mm / 25.4)
        CLICKS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * Math.PI);
    }

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = getHeading() - targetHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double strafe, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        strafeSpeed = strafe;   // save this value
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        double leftFrontPower = (drive - strafe) - turn;
        double rightFrontPower = (drive + strafe) + turn;
        double leftBackPower = (drive + strafe) - turn;
        double rightBackPower = (drive - strafe) + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max;
        List<Double> powerList = new ArrayList<>(
                Arrays.asList(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
        max = Collections.max(powerList);

        if (max > 1.0)
        {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param robotPath  Set to (Straight, Turning, or Strafe)
     *                   the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(String robotPath) {

        if (robotPath.equals("Straight") || robotPath.equals("Strafe")){
            telemetry.addData("Motion", robotPath);

            telemetry.addData("Target Pos LF:RF:LR:RR",
                    Double.parseDouble(JavaUtil.formatNumber(LFPos,0)) + " : " +
                            Double.parseDouble(JavaUtil.formatNumber(RFPos,0)) + " : " +
                            Double.parseDouble(JavaUtil.formatNumber(LRPos,0)) + " : " +
                            Double.parseDouble(JavaUtil.formatNumber(RRPos,0)));

            telemetry.addData("Actual Pos LF:RF:LR:RR",
                    Double.parseDouble(JavaUtil.formatNumber(leftFront.getCurrentPosition(), 0)) + " : " +
                            Double.parseDouble(JavaUtil.formatNumber(rightFront.getCurrentPosition(), 0)) + " : " +
                            Double.parseDouble(JavaUtil.formatNumber(leftBack.getCurrentPosition(), 0)) + " : " +
                            Double.parseDouble(JavaUtil.formatNumber(rightBack.getCurrentPosition(), 0)));
        } else {
            telemetry.addData("Motion", robotPath);
        }

        telemetry.addData("Heading - Target : Current",
                Double.parseDouble(JavaUtil.formatNumber(targetHeading, 2)) + " : " +
                        Double.parseDouble(JavaUtil.formatNumber(getHeading(), 2)));
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        //telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftFrontPower, rightFrontPower);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Turn all motors off
     */
    public void driveMotorsOff() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
