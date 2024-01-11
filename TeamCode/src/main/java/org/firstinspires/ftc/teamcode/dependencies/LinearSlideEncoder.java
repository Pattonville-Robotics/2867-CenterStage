package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlideEncoder {
    LinearOpMode linearOp;
    public DcMotor motor;
    public DcMotor motor2;
    public LinearPosition currentPosition = LinearPosition.ZERO;
    public float analogPos;

    public LinearSlideEncoder (LinearOpMode linearOp) {
        this.linearOp = linearOp;
        HardwareMap hardwareMap = linearOp.hardwareMap;
        motor = hardwareMap.dcMotor.get("mls");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor2 = hardwareMap.dcMotor.get("mls2");
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setTargetPosition(0);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    // Tested heights for junctions, in motor ticks.
    public enum LinearPosition {
        ZERO(10),
        ONE(1400),
        TWO(2300),
        THREE(3200),
        CONE1(700),
        CONE2(120),
        CONE3(180);
        private final int ticks;
        LinearPosition(int i) {this.ticks = i;}
    }

    public void setHeight(LinearPosition pos, double power) {
        motor.setPower(power);
        currentPosition = pos;
        motor.setTargetPosition(pos.ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void makeRobotHang(float magnitude){
        motor.setTargetPosition((int) (motor.getCurrentPosition() + Math.floor(magnitude * 160)));
        motor.setPower(magnitude);
        motor2.setPower(0);
    }

    public void analogMoveSlide(float magnitude) {
        // if slide is going above upper bound (3rd junction height), stop and return early. only stop if slide is moving up.
//        if ((motor.getCurrentPosition() >= LinearPosition.THREE.ticks) && (magnitude > 0)) return;
        if (magnitude != 0) {
            // Disallows movement past the lower bound
            if ((motor.getCurrentPosition() <= LinearPosition.ZERO.ticks) && (magnitude < 0))
                return;
            // (control hub side) pulls the slide up
            if (motor.getCurrentPosition() + 315 < motor2.getCurrentPosition()) {
                motor.setTargetPosition((int) (motor.getCurrentPosition() + Math.floor(magnitude * 160)));
                motor.setPower(magnitude);
            }
            // (battery side) pulls the slide down
            if (motor.getCurrentPosition() + 315 < motor2.getCurrentPosition()) {
                motor2.setTargetPosition((int) (motor.getCurrentPosition() + Math.floor(magnitude * 80)));
                motor2.setPower(magnitude);
            }

        } else {
            // Holds the current position at max power when not moving
            motor.setTargetPosition(motor.getCurrentPosition());
            motor.setPower(1f);
            motor2.setTargetPosition(motor2.getCurrentPosition());
            motor2.setPower(1f);
        }
        analogPos = motor.getCurrentPosition();
    }

    public void setPower(double p){
        motor.setPower(p);
        motor2.setPower(p);
    }

    public double getPower(){
        return motor.getPower();
    }

    public void reset() {
        // Reset the motor's "0" position. Only necessary in case the slide string gets stuck going down to make positions enum accurate.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}