package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.enums.AllianceSide;
//import org.firstinspires.ftc.teamcode.enums.ColorSide;
import org.firstinspires.ftc.teamcode.dependencies.MecanumEncoder;
import org.firstinspires.ftc.teamcode.enums.Direction;
import org.firstinspires.ftc.teamcode.enums.StartPosition;
import org.firstinspires.ftc.teamcode.dependencies.RobotParameters;
//import org.firstinspires.ftc.teamcode.enums.Parking;

public class FirstCenterStage {
    private static DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, linearSlide;
    private static Servo claw;

    private static BNO055IMU imu;

    public static void run(LinearOpMode linearOpMode, StartPosition startPosition) {
        motorFrontLeft = linearOpMode.hardwareMap.dcMotor.get("frontRight");
        motorBackLeft = linearOpMode.hardwareMap.dcMotor.get("backRight");
        motorFrontRight = linearOpMode.hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = linearOpMode.hardwareMap.dcMotor.get("backLeft");
        imu = linearOpMode.hardwareMap.get(BNO055IMU.class, "imu");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        RobotParameters rP = new RobotParameters(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, 28, 7.5, 60, imu, 10.5);
        MecanumEncoder autoEncoder = new MecanumEncoder(rP, linearOpMode);

        switch (startPosition) { // Remember that the collection side (no crossbars) is the "back"
            case FRONT:
                autoEncoder.moveInches(Direction.BACKWARD, 31, 1);
                autoEncoder.moveInches(Direction.FORWARD, 8, 1);
                break;
            case BACKSTAGE_RED: // temporarily set to push 2 pixels to the score zone and park
                autoEncoder.moveInches(Direction.BACKWARD, 41, 1);
                autoEncoder.moveInches(Direction.FORWARD, 8, 1);
                break;
            case BACKSTAGE_BLUE: // same as above just to mitigate confusion
                autoEncoder.moveInches(Direction.BACKWARD, 41, 1);
                autoEncoder.moveInches(Direction.FORWARD, 8, 1);
                break;
            default:
                throw new IllegalArgumentException("StartPosition must be FRONT, BACKSTAGE_BLUE, or BACKSTAGE_RED");
        }

    }
}