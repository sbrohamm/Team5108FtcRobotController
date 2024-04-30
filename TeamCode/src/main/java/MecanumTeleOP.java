package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "mecanum", group = "examples")
public class MecanumTeleOp extends LinearOpMode {

    public DcMotor LeftFront;
    public DcMotor RightFront;
    public DcMotor LeftBack;
    public DcMotor RightBack;


    @Override
    public void runOpMode() throws interruptedException {

        LeftFront = hardwareMap.dcMotor.get("Leftfront");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        waitForStart();

        while(opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double turn = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

            LeftFront.setPower( (y + x + turn) / denominator);
            RightFront.setPower( (y - x + turn) / denominator);
            LeftBack.setPower( (y - x - turn) / denominator);
            RightBack.setPower( (y + x - turn) / denominator);
        }

    }

}