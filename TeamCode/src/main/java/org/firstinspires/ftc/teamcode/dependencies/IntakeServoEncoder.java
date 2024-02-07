package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
public class IntakeServoEncoder {
    LinearOpMode linearOp;
    // Servo on the side of the robot with the battery
    public CRServo servoB;
    // Servo on the side of the robot with the hubs
    public CRServo servoH;

    public IntakeServoEncoder (LinearOpMode linearOp) {
        this.linearOp = linearOp;
        HardwareMap hardwareMap = linearOp.hardwareMap;
        servoB = hardwareMap.get(CRServo.class, "batSideIntake");
        servoH = hardwareMap.get(CRServo.class, "hubSideIntake");
    }
    public void rotateServos(double power) {
        servoH.setPower(power);
        servoB.setPower(-power);
    }
}
