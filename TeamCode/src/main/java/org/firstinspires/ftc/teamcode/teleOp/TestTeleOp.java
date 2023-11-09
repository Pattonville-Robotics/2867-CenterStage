package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class TestTeleOp extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        // Declare our motors
        // Make sure your ID's match your configuration
        // TODO: change hardware IDs to match variable names
        // TODO: fix drift by tweaking some values
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontLeft"); // front right
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backLeft");   // back right
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontRight"); // front left
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backRight");   // back left

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()){
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (Math.pow((y + x + rx),3) * 0.4) / denominator;
            double backLeftPower = (Math.pow((y - x + rx),3) * 0.4) / denominator;
            double frontRightPower = (Math.pow((y - x - rx),3) * 0.4) / denominator;
            double backRightPower = -(Math.pow((y + x - rx),3) * 0.4) / denominator;
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
    }
}
