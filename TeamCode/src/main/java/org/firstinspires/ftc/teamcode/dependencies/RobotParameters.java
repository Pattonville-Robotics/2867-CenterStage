package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotParameters {
    private BNO055IMU imu;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, linearSlide;
    public int ticks;
    public double wheelRadius, driveGearRatio, wheelCircumference, wheelBaseCircumference, wheelBaseRadius;

    public RobotParameters(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, int t, double wheelRadius, double driveGearRatio, BNO055IMU imu, double wheelBaseRadius){
        this.frontRightMotor = fr;
        this.backRightMotor = br;
        this.frontLeftMotor = fl;
        this.backLeftMotor = bl;
        this.ticks = t;
        this.wheelRadius = wheelRadius;
        this.driveGearRatio = driveGearRatio;

        this.wheelCircumference = Math.PI*2*wheelRadius;

//        this.linearSlide = LS;
        this.wheelBaseRadius = wheelBaseRadius;
        this.wheelBaseCircumference = (2*Math.PI*wheelBaseRadius);
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        this.imu = imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public RobotParameters(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, int t, double wheelRadius, double driveGearRatio, DcMotor LS, double wheelBaseRadius){
        this.frontRightMotor = fr;
        this.backRightMotor = br;
        this.frontLeftMotor = fl;
        this.backLeftMotor = bl;
        this.ticks = t;
        this.wheelRadius = wheelRadius;
        this.driveGearRatio = driveGearRatio;
        this.wheelCircumference = Math.PI*2*wheelRadius;
        this.linearSlide = LS;
        this.wheelBaseRadius = wheelBaseRadius;
    }
    public double getHeading(){
        return this.imu.getAngularOrientation().firstAngle;
    }




}