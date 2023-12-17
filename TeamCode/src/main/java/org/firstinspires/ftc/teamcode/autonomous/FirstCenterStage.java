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
import org.firstinspires.ftc.teamcode.dependencies.MecanumEncoder;
import org.firstinspires.ftc.teamcode.dependencies.PositionVector;
import org.firstinspires.ftc.teamcode.enums.Direction;
import org.firstinspires.ftc.teamcode.enums.StartPosition;
import org.firstinspires.ftc.teamcode.dependencies.RobotParameters;
//import org.firstinspires.ftc.teamcode.enums.Parking;


public class FirstCenterStage {
    private static DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, linearSlide;
    private static Servo claw;


    private static IMU imu;


    public static void run(LinearOpMode linearOpMode, StartPosition startPosition) {
        motorFrontLeft = linearOpMode.hardwareMap.dcMotor.get("frontRight");
        motorBackLeft = linearOpMode.hardwareMap.dcMotor.get("backRight");
        motorFrontRight = linearOpMode.hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = linearOpMode.hardwareMap.dcMotor.get("backLeft");


        RevColorSensorV3 cSensorL = linearOpMode.hardwareMap.get(RevColorSensorV3.class, "cSensorL");
        RevColorSensorV3 cSensorR = linearOpMode.hardwareMap.get(RevColorSensorV3.class, "cSensorR");


        double Left1;
        double Left2;
        double Right1;
        double Right2;


        IMU imu = linearOpMode.hardwareMap.get(IMU.class, "imu");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        RobotParameters rP = new RobotParameters(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, 28, 7.5, 60, imu, 10.5);
        MecanumEncoder autoEncoder = new MecanumEncoder(rP, linearOpMode);
//        autoEncoder.newRotateDegrees(40,0.5, Direction.CW);
//        autoEncoder.rotateDegrees(Direction.CW,40, 0.5);
//        autoEncoder.travelTo(new PositionVector(24, 24, 60), 0.6, 0.8);


        // The code below is commented for now so that the code above can be tested without complications


        switch (startPosition) { // Remember that the collection side (no crossbars) is the "back"
            case FRONT:
                autoEncoder.moveInches(Direction.BACKWARD, 26, 1);

                autoEncoder.moveInches(Direction.LEFT, 2, 1);

                Left1 = cSensorL.getDistance(DistanceUnit.CM);
                Right1 = cSensorR.getDistance(DistanceUnit.CM);

                autoEncoder.moveInches(Direction.RIGHT, 4, 1);

                Left2 = cSensorL.getDistance(DistanceUnit.CM);
                Right2 = cSensorR.getDistance(DistanceUnit.CM);

                autoEncoder.moveInches(Direction.LEFT, 2, 1);
                if (Left1 < 5 && Right1 > 5){
                    autoEncoder.moveInches(Direction.LEFT, 15, 1);
                    autoEncoder.moveInches(Direction.RIGHT, 3, 1);
                } else if (Right2 < 5 && Left2 > 5){
                    autoEncoder.moveInches(Direction.RIGHT, 15, 1);
                    autoEncoder.moveInches(Direction.LEFT, 3, 1);
                }else {
                    autoEncoder.moveInches(Direction.BACKWARD, 5, 1);
                    autoEncoder.moveInches(Direction.FORWARD, 8, 1);
                }
                break;
            case BACKSTAGE_RED: // temporarily set to push 2 pixels to the score zone and park
                autoEncoder.moveInches(Direction.RIGHT, 2, 1);
                autoEncoder.moveInches(Direction.LEFT, 2, 1);
                autoEncoder.moveInches(Direction.BACKWARD, 26, 1);

                autoEncoder.moveInches(Direction.LEFT, 2, 1);

                Left1 = cSensorL.getDistance(DistanceUnit.CM);
                Right1 = cSensorR.getDistance(DistanceUnit.CM);

                autoEncoder.moveInches(Direction.RIGHT, 4, 1);

                Left2 = cSensorL.getDistance(DistanceUnit.CM);
                Right2 = cSensorR.getDistance(DistanceUnit.CM);

                autoEncoder.moveInches(Direction.LEFT, 2, 1);

                if (Left1 < 5 && Right1 > 5){
                    autoEncoder.moveInches(Direction.LEFT, 15, 1);
                    autoEncoder.moveInches(Direction.RIGHT, 3, 1);
                } else if (Right2 < 5 && Left2 > 5){
                    autoEncoder.moveInches(Direction.RIGHT, 15, 1);
                    autoEncoder.moveInches(Direction.LEFT, 3, 1);
                    autoEncoder.moveInches(Direction.FORWARD, 10, 1);
                    autoEncoder.moveInches(Direction.LEFT, 12, 1);
                    autoEncoder.moveInches(Direction.FORWARD, 15, 1);
                    autoEncoder.moveInches(Direction.RIGHT, 36, 1);
                }else {
                    autoEncoder.moveInches(Direction.BACKWARD, 5, 1);
                    autoEncoder.moveInches(Direction.FORWARD, 28, 1);
                    autoEncoder.moveInches(Direction.RIGHT, 40, 1);
                }
                break;
            case BACKSTAGE_BLUE: // same as above just to mitigate confusion
                autoEncoder.moveInches(Direction.LEFT, 2, 1);
                autoEncoder.moveInches(Direction.RIGHT, 2, 1);
                autoEncoder.moveInches(Direction.BACKWARD, 26, 1);

                autoEncoder.moveInches(Direction.LEFT, 2, 1);

                Left1 = cSensorL.getDistance(DistanceUnit.CM);
                Right1 = cSensorR.getDistance(DistanceUnit.CM);

                autoEncoder.moveInches(Direction.RIGHT, 4, 1);

                Left2 = cSensorL.getDistance(DistanceUnit.CM);
                Right2 = cSensorR.getDistance(DistanceUnit.CM);

                autoEncoder.moveInches(Direction.LEFT, 2, 1);

                if (Left1 < 5 && Right1 > 5){
                    autoEncoder.moveInches(Direction.LEFT, 15, 1);
                    autoEncoder.moveInches(Direction.RIGHT, 3, 1);
                    autoEncoder.moveInches(Direction.FORWARD, 10, 1);
                    autoEncoder.moveInches(Direction.RIGHT, 12, 1);
                    autoEncoder.moveInches(Direction.FORWARD, 15, 1);
                    autoEncoder.moveInches(Direction.LEFT, 36, 1);
                } else if (Right2 < 5 && Left2 > 5){
                    autoEncoder.moveInches(Direction.RIGHT, 15, 1);
                    autoEncoder.moveInches(Direction.LEFT, 3, 1);
                }else {
                    autoEncoder.moveInches(Direction.BACKWARD, 5, 1);
                    autoEncoder.moveInches(Direction.FORWARD, 28, 1);
                    autoEncoder.moveInches(Direction.LEFT, 40, 1);
                }
                break;
            default:
                throw new IllegalArgumentException("StartPosition must be FRONT, BACKSTAGE_BLUE, or BACKSTAGE_RED");
        }
    }
}

