package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.dependencies.IntakeServoEncoder;
import org.firstinspires.ftc.teamcode.dependencies.LinearSlideEncoder;
@TeleOp
public class CenterstageTeleOp extends LinearOpMode{
    boolean hanging = false;
    @Override
    public void runOpMode() throws InterruptedException{
        // Declare our motors
        // Make sure your ID's match your configuration
        // TODO: change hardware IDs to match variable names
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontLeft"); // front right
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backLeft");   // back right
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontRight"); // front left
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backRight");// back left
        LinearSlideEncoder slide = new LinearSlideEncoder(this);
        IntakeServoEncoder intake = new IntakeServoEncoder(this);

        RevColorSensorV3 cSensorL = hardwareMap.get(RevColorSensorV3.class, "cSensorL");
        RevColorSensorV3 cSensorR = hardwareMap.get(RevColorSensorV3.class, "cSensorR");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                new Orientation(
                        AxesReference.INTRINSIC,
                        AxesOrder.ZYX,
                        AngleUnit.DEGREES,
                        0,
                        0,
                        0,
                        0  // acquisitionTime, not used
                )
        ));
        imu.initialize(parameters);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()){
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            double powerMult = 0.5 + 0.5 * gamepad1.right_trigger;
            double position = 0;
//            if (gamepad1.a) {
//                imu.resetYaw();
//            }
            float lsPower = 0;
            if (gamepad1.x) {
                lsPower = -1f;
            } else if (gamepad1.y) {
                lsPower = 1f;
            }
            if (gamepad1.a) {
                hanging = false;
            }
            if (gamepad1.b) {
                hanging = true;
            }
            if(gamepad1.left_bumper) {
                intake.rotateServos(0.7);
            } else if(gamepad1.right_bumper) {
                intake.rotateServos(-0.7);
            } else {
                intake.rotateServos(0);
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = -((rotY + rotX + rx) * powerMult) / denominator;
            double backLeftPower = -((rotY - rotX + rx) * powerMult) / denominator;
            double frontRightPower = -((rotY - rotX - rx) * powerMult) / denominator;
            double backRightPower = ((rotY + rotX - rx) * powerMult) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            if (hanging) {
                slide.analogMoveSlide(-1f);
            } else {
                slide.analogMoveSlide(lsPower);
            }

            telemetry.addData("Left Sensor Distance", cSensorL.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Sensor Distance", cSensorR.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }
    }
}
