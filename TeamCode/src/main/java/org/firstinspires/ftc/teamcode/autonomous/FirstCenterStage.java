package org.firstinspires.ftc.teamcode.autonomous;




import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//import org.firstinspires.ftc.teamcode.enums.AllianceSide;
//import org.firstinspires.ftc.teamcode.enums.ColorSide;
import org.firstinspires.ftc.teamcode.dependencies.IntakeServoEncoder;
import org.firstinspires.ftc.teamcode.dependencies.MecanumEncoder;
import org.firstinspires.ftc.teamcode.dependencies.PositionVector;
import org.firstinspires.ftc.teamcode.enums.Direction;
import org.firstinspires.ftc.teamcode.enums.StartPosition;
import org.firstinspires.ftc.teamcode.dependencies.RobotParameters;

import java.util.concurrent.TimeUnit;
//import org.firstinspires.ftc.teamcode.enums.Parking;


public class FirstCenterStage {
    private static DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, linearSlide;
    private static Servo claw;
    private static IMU imu;

    public static void placePurplePixel(LinearOpMode linearOp, MecanumEncoder mecEncoder, IntakeServoEncoder intEncoder, RevColorSensorV3 cL, RevColorSensorV3 cR) {
        mecEncoder.moveInches(Direction.FORWARD, 26, 1);
        double leftDist = cL.getDistance(DistanceUnit.INCH);
        double rightDist = cR.getDistance(DistanceUnit.INCH);
        if (leftDist < 2) { // Left spike mark
            mecEncoder.rotateDegrees(Direction.CCW, 90, 1);
            mecEncoder.moveInches(Direction.RIGHT, 2, 1);
            mecEncoder.moveInches(Direction.FORWARD, 2, 1);
        } else if (rightDist < 2) { // Right spike mark
            mecEncoder.rotateDegrees(Direction.CW, 90, 1);
            mecEncoder.moveInches(Direction.LEFT, 2, 1);
            mecEncoder.moveInches(Direction.FORWARD, 2, 1);
        } else { // Center spike mark
            mecEncoder.moveInches(Direction.FORWARD, 5, 1);
            mecEncoder.moveInches(Direction.BACKWARD, 3, 1);
        }
        intEncoder.rotateServos(0.5);
        linearOp.sleep(1500);
        intEncoder.rotateServos(0);

        mecEncoder.moveInches(Direction.BACKWARD, 6, 1);
        if (leftDist < 2) {
            mecEncoder.rotateDegrees(Direction.CW, 90, 1);
            mecEncoder.moveInches(Direction.LEFT, 4.5, 1);
            mecEncoder.moveInches(Direction.BACKWARD, 6, 1);
        } else if (rightDist < 2) {
            mecEncoder.rotateDegrees(Direction.CCW, 90, 1);
            mecEncoder.moveInches(Direction.RIGHT, 4.5, 1);
            mecEncoder.moveInches(Direction.BACKWARD, 6, 1);
        }
        mecEncoder.moveInches(Direction.BACKWARD, 18, 1);
    }

    public static void run(LinearOpMode linearOpMode, StartPosition startPosition) {
        motorFrontLeft = linearOpMode.hardwareMap.dcMotor.get("frontRight");
        motorBackLeft = linearOpMode.hardwareMap.dcMotor.get("backRight");
        motorFrontRight = linearOpMode.hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = linearOpMode.hardwareMap.dcMotor.get("backLeft");


        RevColorSensorV3 cSensorL = linearOpMode.hardwareMap.get(RevColorSensorV3.class, "cSensorL");
        RevColorSensorV3 cSensorR = linearOpMode.hardwareMap.get(RevColorSensorV3.class, "cSensorR");

        IMU imu = linearOpMode.hardwareMap.get(IMU.class, "imu");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        RobotParameters rP = new RobotParameters(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, 28, 7.5, 60, imu, 10.5);
        MecanumEncoder autoEncoder = new MecanumEncoder(rP, linearOpMode);
        IntakeServoEncoder intake = new IntakeServoEncoder(linearOpMode);
//        autoEncoder.travelTo(new PositionVector(24, 24, 60), 0.6, 0.8);
        switch (startPosition) { // Remember that the collection side (no crossbars) is the "back"
            case FRONT:
                placePurplePixel(linearOpMode, autoEncoder, intake, cSensorL, cSensorR);

                break;
            case BACKSTAGE_RED: // temporarily set to push 2 pixels to the score zone and park
                placePurplePixel(linearOpMode, autoEncoder, intake, cSensorL, cSensorR);
                autoEncoder.rotateDegrees(Direction.CW, 90, 1);
                autoEncoder.moveInches(Direction.FORWARD, 32, 1);
                intake.rotateServos(0.5);
                autoEncoder.moveInches(Direction.BACKWARD, 5, 1);
                intake.rotateServos(0);
                break;
            case BACKSTAGE_BLUE: // same as above just to mitigate confusion
                placePurplePixel(linearOpMode, autoEncoder, intake, cSensorL, cSensorR);
                autoEncoder.rotateDegrees(Direction.CCW, 90, 1);
                autoEncoder.moveInches(Direction.FORWARD, 32, 1);
                intake.rotateServos(0.5);
                autoEncoder.moveInches(Direction.BACKWARD, 5, 1);
                intake.rotateServos(0);
                break;
            default:
                throw new IllegalArgumentException("StartPosition must be FRONT, BACKSTAGE_BLUE, or BACKSTAGE_RED");
        }
    }
}

