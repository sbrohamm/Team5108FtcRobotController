package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Autonomous(name = "Two Wheel Odometry With Gyro", group="Robot")
public class TwoWheelOdometryWithGyro extends LinearOpMode{
    // Adjust these numbers to suit your robot.
    private final double ODOM_INCHES_PER_COUNT   = 0.002968;   //  GoBilda Odometry Pod, 48mm wheel dia., 2000 ticks/revolution, (1/326.8)
    private final boolean INVERT_DRIVE_ODOMETRY  = false;      //  When driving FORWARD, the odometry value MUST increase.  If it does not, flip the value of this constant.
    private final boolean INVERT_STRAFE_ODOMETRY = true;       //  When strafing to the LEFT, the odometry value MUST increase.  If it does not, flip the value of this constant.

    private static final double DRIVE_GAIN          = 0.03;    // Strength of axial position control, smaller is slower to position, larger is faster to position
    private static final double DRIVE_ACCEL         = 2.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE     = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double DRIVE_MAX_AUTO      = 0.6;     // "default" Maximum Axial power limit during autonomous

    private static final double STRAFE_GAIN         = 0.025;    // Strength of lateral position control, smaller is slower to position, larger is faster to position
    private static final double STRAFE_ACCEL        = 1.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double STRAFE_TOLERANCE    = 0.4;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double STRAFE_DEADBAND     = 0.2;     // Error less than this causes zero output.  Must be smaller than STRAFE_TOLERANCE
    private static final double STRAFE_MAX_AUTO     = 0.6;     // "default" Maximum Lateral power limit during autonomous

    private static final double YAW_GAIN            = 0.018;    // Strength of Yaw position control, smaller is slower to position, larger is faster to position
    private static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than YAW_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    // Public Members
    public double driveDistance     = 0;   // scaled axial distance in inches (+ = forward)
    public double strafeDistance    = 0;   // scaled lateral distance in inches (+ = left)
    public double heading           = 0;   // Latest Robot heading from IMU
    public double directionInches   = 0;   // Intuitive method to change distance +/- using String "Forward", "Backward", "Left", "Right"


    // ---  Private Members

    // Hardware interface Objects
    private DcMotor leftFrontDrive;     //  control the left front drive wheel
    private DcMotor rightFrontDrive;    //  control the right front drive wheel
    private DcMotor leftBackDrive;      //  control the left back drive wheel
    private DcMotor rightBackDrive;     //  control the right back drive wheel

    private DcMotor driveEncoder;       //  the Axial (front/back) Odometry Module (may overlap with motor, or may not)
    private DcMotor strafeEncoder;      //  the Lateral (left/right) Odometry Module (may overlap with motor, or may not)

    private IMU imu;
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    private int rawDriveOdometer    = 0; // Unmodified axial odometer count
    private int driveOdometerOffset = 0; // Used to offset axial odometer, positive increases distance, negative decreases distance
    private int rawStrafeOdometer   = 0; // Unmodified lateral odometer count
    private int strafeOdometerOffset= 0; // Used to offset lateral odometer, positive increases distance, negative decreases distance
    private double rawHeading       = 0; // Unmodified current heading (degrees) obtained from IMU
    private double headingOffset    = 0; // Used to offset heading, positive increases angle turned, negative decreases angle

    private double turnRate         = 0; // Latest Robot Turn Rate from IMU
    private boolean showTelemetry   = true; // Display feedback information to driver station

    /**
     * This OpMode defines the robot's ability of driving an autonomous path using two encoder wheels
     * with IMU Gyro heading correction.
     * The path to be followed is built from a series of drive, strafe, turn, or pause steps.
     * This code uses RUN_USING_ENCODER for drive, strafe, turning, and holding position.
     */
    @Override
    public void runOpMode () {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBack");

        initializeRobot(true);

        while (opModeInInit()) {
            readSensors();
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.addData(">", "Touch Play to run Auto");
            telemetry.update();
        }

        waitForStart();
        resetHeading();  // Reset heading to set a baseline for Auto
        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // Note, this example takes more than 30 seconds to execute, so turn OFF the auto timer.

            // Drive a large rectangle, turning at each corner
            driveRobot("Forward",  20, 0.60, 0.25);
            turnToHeading(90, 0.45, 0.5);
            driveRobot("Forward",  20, 0.60, 0.25);
            turnToHeading(180, 0.45, 0.5);
            driveRobot("Forward",  20, 0.60, 0.25);
            turnToHeading(270, 0.45, 0.5);
            driveRobot("Forward",  20, 0.60, 0.25);
            turnToHeading(0, 0.45, 0.5);

            sleep(500);

            // Drive the path again without turning.
            driveRobot("Forward",  20, 0.60, 0.15);
            strafeRobot("Left", 20, 0.60, 0.15);
            driveRobot("Backward", 20, 0.60, 0.15);
            strafeRobot("Right",20, 0.60, 0.15);

        }
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initializeRobot(boolean showTelemetry)
    {
        /*
         To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
         When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
         Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        */

        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Place motors into braking mode so motors stop abruptly with zero power
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  Connect to the encoder channels using the name of that channel.
        driveEncoder = hardwareMap.get(DcMotor.class, "AxialOdoPod");
        strafeEncoder = hardwareMap.get(DcMotor.class, "LateralOdoPod");

        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Specify how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // zero out all the odometry readings.
        resetOdometry();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    /**
     * Read all input devices to determine the robot's motion
     * always return true so this can be used in "while" loop conditions
     * Set sensorsRead to true for data inclusion in readouts.
     */
    public boolean readSensors() {
        rawDriveOdometer = driveEncoder.getCurrentPosition() * (INVERT_DRIVE_ODOMETRY ? -1 : 1);
        rawStrafeOdometer = strafeEncoder.getCurrentPosition() * (INVERT_STRAFE_ODOMETRY ? -1 : 1);
        driveDistance = (rawDriveOdometer - driveOdometerOffset) * ODOM_INCHES_PER_COUNT;
        strafeDistance = (rawStrafeOdometer - strafeOdometerOffset) * ODOM_INCHES_PER_COUNT;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        rawHeading  = orientation.getYaw(AngleUnit.DEGREES);
        heading     = rawHeading - headingOffset;
        turnRate    = angularVelocity.zRotationRate;

        if (showTelemetry) {
            telemetry.addData("Odom Ax:Lat", "%6d %6d", rawDriveOdometer - driveOdometerOffset, rawStrafeOdometer - strafeOdometerOffset);
            telemetry.addData("Dist Ax:Lat", "%5.2f %5.2f", driveDistance, strafeDistance);
            telemetry.addData("Head Deg:Rate", "%5.2f %5.2f", heading, turnRate);
        }
        //sensorsRead = true; // Do this so this function can be included in the condition for a while loop to keep values fresh.
        return true;
    }

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param direction is either "Forward" (positive power) or "Backward" (negative power)
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse, converted into "directionInches"
     * @param drivePower Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void driveRobot(String direction, double distanceInches, double drivePower, double holdTime) {
        if (direction.equals("Forward")){
            directionInches = (distanceInches * 1);
        }else if (direction.equals("Backward")) {
            directionInches = (distanceInches * -1);
        }
        resetOdometry();
        driveControlSet2Zero(0);
        driveControlNewDistance(directionInches, drivePower);   // achieve desired drive distance
        strafeControlSet2Zero(0);              // Maintain zero strafe drift
        yawControlReset();                          // Maintain last turn angle
        holdTimer.reset();

        while (opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveControlGetPower(driveDistance), strafeControlGetPower(strafeDistance), yawControlGetPower(heading));

            // Time to exit?
            if (driveControlInPosition() && yawControlInPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            sleep(10);
        }
        stopRobot();
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param direction is either "Forward" (positive power) or "Backward" (negative power)
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse, converted into "directionInches"
     * @param strafePower Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void strafeRobot(String direction, double distanceInches, double strafePower, double holdTime) {
        if (direction.equals("Left")){
            directionInches = (distanceInches * 1);
        }else if (direction.equals("Right")) {
            directionInches = (distanceInches * -1);
        }

        resetOdometry();
        driveControlSet2Zero(0);             //  Maintain zero drive drift
        strafeControlSet2Zero(0);
        strafeControlNewDistance(directionInches, strafePower);  // Achieve desired Strafe distance
        yawControlReset();                          // Maintain last turn angle
        holdTimer.reset();

        while (opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveControlGetPower(driveDistance), strafeControlGetPower(strafeDistance), yawControlGetPower(heading));

            // Time to exit?
            if (strafeControlInPosition() && yawControlInPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            sleep(10);
        }
        stopRobot();
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param drivePower Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void turnToHeading(double headingDeg, double drivePower, double holdTime) {

        yawControlSet2Zero(0);
        yawControlNewHeading(headingDeg, drivePower);

        while (opModeIsActive() && readSensors()) {

            // implement desired axis powers
            moveRobot(0, 0, yawControlGetPower(heading));

            // Time to exit?
            if (yawControlInPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            sleep(10);
        }
        stopRobot();
    }

    //  ########################  Low level control functions.  ###############################

    /**
     * Drive the wheel motors to obtain the requested axes motions
     * @param drive     Fwd/Rev axis power
     * @param strafe    Left/Right axis power
     * @param yaw       Yaw axis power
     */
    public void moveRobot(double drive, double strafe, double yaw){

        double leftFrontPower = drive - strafe - yaw;
        double rightFrontPower = drive + strafe + yaw;
        double leftBackPower = drive + strafe - yaw;
        double rightBackPower = drive - strafe + yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        //normalize the motor values
        if (max > 1.0)  {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        //send power to the motors
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (showTelemetry) {
            telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
            telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.update(); //  Assume this is the last thing done in the loop.
        }
    }

    /**
     * Stop all motors.
     */
    public void stopRobot() {
        moveRobot(0,0,0);
    }

    /**
     * Set odometry counts and distances to zero.
     */
    public void resetOdometry() {
        readSensors();
        driveOdometerOffset = rawDriveOdometer;
        driveDistance = 0.0;
        driveControlSet2Zero(0);

        strafeOdometerOffset = rawStrafeOdometer;
        strafeDistance = 0.0;
        strafeControlSet2Zero(0);
    }

    /**
     * Reset the robot heading to zero degrees, and also lock that heading into heading controller.
     */
    public void resetHeading() {
        readSensors();
        headingOffset = rawHeading;
        yawControlSet2Zero(0);
        heading = 0;
    }

    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    /**
     * Set the drive telemetry on or off
     */
    public void showTelemetry(boolean show){
        showTelemetry = show;
    }


    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    // Drive Proportional Controller variables
    double  driveLastOutput          = 0.0;
    double  driveLiveOutputLimit    = 0;
    double  driveSetPoint            = 0;
    boolean driveInPosition;
    ElapsedTime driveCycleTime  = new ElapsedTime();

    // Strafe Proportional Controller variables
    double  strafeLastOutput          = 0.0;
    double  strafeLiveOutputLimit    = 0;
    double  strafeSetPoint            = 0;
    boolean strafeInPosition;
    ElapsedTime strafeCycleTime = new ElapsedTime();

    // Yaw Proportional Controller variables
    double  yawLastOutput          = 0.0;
    double  yawLiveOutputLimit    = 0;
    double  yawSetPoint            = 0;
    boolean yawInPosition;
    ElapsedTime yawCycleTime    = new ElapsedTime();

    /**
     * Saves a new setpoint (destination) and sets a new output power.
     * This call allows a temporary power limit to be set to override the default.
     *      @param setPoint new destination point
     *      @param powerLimit new powerLimit
     */
    private void driveControlNewDistance(double setPoint, double powerLimit) {
        driveLiveOutputLimit = Math.abs(powerLimit);
        driveSetPoint = setPoint; //SetPoint is the distance to travel either forward or backward
        driveControlReset();
    }
    private void strafeControlNewDistance(double setPoint, double powerLimit) {
        strafeLiveOutputLimit = Math.abs(powerLimit);
        strafeSetPoint = setPoint; //SetPoint is the distance to strafe either right or left
        strafeControlReset();
    }
    private void yawControlNewHeading(double setPoint, double powerLimit) {
        yawLiveOutputLimit = Math.abs(powerLimit);
        yawSetPoint = setPoint;  //SetPoint is the desired heading to turn to
        yawControlReset();
    }
    /**
     * Resets the setpoint to zero and resets the output power history to the CONSTANT MAX
     */
    private void driveControlSet2Zero(int setPoint) {
        driveLiveOutputLimit = DRIVE_MAX_AUTO;
        driveSetPoint = setPoint;
        driveControlReset();
    }
    private void strafeControlSet2Zero(int setPoint) {
        strafeLiveOutputLimit = STRAFE_MAX_AUTO;
        strafeSetPoint = setPoint;
        strafeControlReset();
    }
    private void yawControlSet2Zero(int setPoint) {
        yawLiveOutputLimit = YAW_MAX_AUTO;
        yawSetPoint = setPoint;
        yawControlReset();
    }
    /**
     * Restart the acceleration timer and set output (destination) to 0
     */
    private void driveControlReset() {
        driveCycleTime.reset();
        driveInPosition = false;
        driveLastOutput = 0.0;
    }
    private void strafeControlReset() {
        strafeCycleTime.reset();
        strafeInPosition = false;
        strafeLastOutput = 0.0;
    }
    private void yawControlReset() {
        yawCycleTime.reset();
        yawInPosition = false;
        yawLastOutput = 0.0;
    }

    /**
     * Calculate the required power to move the robot to the requested destination
     * @param input is the desired distance to travel forward or backward
     * @return driveOutput - the output power needed to achieve the desired distance
     */
    public double driveControlGetPower(double input) {
        double driveError = driveSetPoint - input;
        double driveDV = driveCycleTime.seconds() * DRIVE_ACCEL;
        double driveOutput;

        if (Math.abs(driveError) < DRIVE_TOLERANCE) {
            driveInPosition = true;
        }

        // Prevent any very slow motor output accumulation
        if (Math.abs(driveError) <= DRIVE_DEADBAND) {
            driveOutput = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            driveOutput = (driveError * DRIVE_GAIN);
            driveOutput = Range.clip(driveOutput, -driveLiveOutputLimit, driveLiveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((driveOutput - driveLastOutput) > driveDV) {
                driveOutput = driveLastOutput + driveDV;
            } else if ((driveOutput - driveLastOutput) < -driveDV) {
                driveOutput = driveLastOutput - driveDV;
            }
        }
        driveLastOutput = driveOutput;
        driveCycleTime.reset();
        return driveOutput;
    }

    /**
     * Calculate the required power to move the robot to the requested destination
     * @param input is the desired distance to travel sideways
     * @return strafeOutput - the output power needed to achieve the desired distance
     */
    public double strafeControlGetPower(double input) {
        double strafeError = strafeSetPoint - input;
        double strafeDV = strafeCycleTime.seconds() * STRAFE_ACCEL;
        double strafeOutput;

        if (Math.abs(strafeError) < STRAFE_TOLERANCE) {
            strafeInPosition = true;
        }

        // Prevent any very slow motor output accumulation
        if (Math.abs(strafeError) <= STRAFE_DEADBAND) {
            strafeOutput = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            strafeOutput = (strafeError * STRAFE_GAIN);
            strafeOutput = Range.clip(strafeOutput, -strafeLiveOutputLimit, strafeLiveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((strafeOutput - strafeLastOutput) > strafeDV) {
                strafeOutput = strafeLastOutput + strafeDV;
            } else if ((strafeOutput - strafeLastOutput) < -strafeDV) {
                strafeOutput = strafeLastOutput - strafeDV;
            }
        }
        strafeLastOutput = strafeOutput;
        strafeCycleTime.reset();
        return strafeOutput;
    }

    /**
     * Calculate the required power to turn the robot to the requested heading
     * @param input is the desired heading to turn robot
     * @return yawOutput - the output power needed to achieve the desired heading
     */
    public double yawControlGetPower(double input) {
        double yawError = yawSetPoint - input;
        double yawDV = yawCycleTime.seconds() * YAW_ACCEL;
        double yawOutput;

        // normalize to +/- 180 when we are controlling heading
        while (yawError > 180)  yawError -= 360;
        while (yawError <= -180) yawError += 360;

        if (Math.abs(yawError) < YAW_TOLERANCE) {
            yawInPosition = true;
        }

        // Prevent any very slow motor output accumulation
        if (Math.abs(yawError) <= YAW_DEADBAND) {
            yawOutput = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            yawOutput = (yawError * YAW_GAIN);
            yawOutput = Range.clip(yawOutput, -yawLiveOutputLimit, yawLiveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((yawOutput - yawLastOutput) > yawDV) {
                yawOutput = yawLastOutput + yawDV;
            } else if ((yawOutput - yawLastOutput) < -yawDV) {
                yawOutput = yawLastOutput - yawDV;
            }
        }
        yawLastOutput = yawOutput;
        yawCycleTime.reset();
        return yawOutput;
    }

    // Query the inPosition variable to determine if the robot is in position
    public boolean driveControlInPosition(){
        return driveInPosition;
    }
    public boolean strafeControlInPosition(){
        return strafeInPosition;
    }
    public boolean yawControlInPosition(){
        return yawInPosition;
    }

}

