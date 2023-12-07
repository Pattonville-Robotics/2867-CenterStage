package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.dependencies.AprilTagDetectionPipeline;

import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class CenterstageTeleOp extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        // Declare our motors
        // Make sure your ID's match your configuration
        // TODO: change hardware IDs to match variable names
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontLeft"); // front right
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backLeft");   // back right
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontRight"); // front left
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backRight");   // back left

        RevColorSensorV3 cSensor = hardwareMap.get(RevColorSensorV3.class, "colorV3");
        DistanceSensor cSensor2 = hardwareMap.get(DistanceSensor.class, "colorV2");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                new Orientation(
                        AxesReference.INTRINSIC,
                        AxesOrder.ZYX,
                        AngleUnit.DEGREES,
                        -90,
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
            double rx = gamepad1.right_stick_x;
            double power_mult = 0.4+(gamepad1.right_trigger/(7/4));
            if (gamepad1.left_bumper) {
                imu.resetYaw();
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
            double frontLeftPower = -((rotY + rotX + rx) * power_mult) / denominator;
            double backLeftPower = -((rotY - rotX + rx) * power_mult) / denominator;
            double frontRightPower = -((rotY - rotX - rx) * power_mult) / denominator;
            double backRightPower = ((rotY + rotX - rx) * power_mult) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addData("Sensor V2 Distance", cSensor2.getDistance(DistanceUnit.METER));
            telemetry.addData("Sensor V3 Distance", cSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
    }
}
