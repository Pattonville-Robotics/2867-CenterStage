package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.enums.AllianceSide;
//import org.firstinspires.ftc.teamcode.enums.ColorSide;
import org.firstinspires.ftc.teamcode.enums.Direction;
import org.firstinspires.ftc.teamcode.dependencies.RobotParameters;
//import org.firstinspires.ftc.teamcode.enums.Parking;

public class FirstCenterStage {
    private static DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, linearSlide;
    private static Servo claw;

    private static BNO055IMU imu;

    public static void run(LinearOpMode linearOpMode) {
        motorFrontLeft = linearOpMode.hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = linearOpMode.hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = linearOpMode.hardwareMap.dcMotor.get("frontRight");
        motorBackRight = linearOpMode.hardwareMap.dcMotor.get("backRight");
        imu = linearOpMode.hardwareMap.get(BNO055IMU.class, "imu");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        RobotParameters rP = new RobotParameters(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, 28, 7.5, 60, imu, 10.5);
    }
}