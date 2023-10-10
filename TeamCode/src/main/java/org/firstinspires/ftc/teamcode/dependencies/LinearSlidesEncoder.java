package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class LinearSlidesEncoder {
    private RobotParameters rP;
    private DcMotor linearSlideMotor;
    private Servo claw;
    private LinearOpMode linearOp;
    public LinearPosition currentLinearPosition = LinearPosition.ZERO;
    public boolean isOpen = true;


    // math.pi*2
    // Heights for da thingies: 37 in, 25 in, 17 in

    //    private final int t = 1440;
//    private final double MIN_POS = 0, MAX_POS = 1;private final int t = 1440;
    private final double MIN_POS = 0, MAX_POS = 1;

    public enum LinearPosition {
        ZERO(0), ONE(1765), TWO(2825), THREE(3930), CONE5(756), CONE4(700), CONE3(605), CONE2(285);
        private final int ticks;
        LinearPosition(int i){this.ticks = i;}
    }

    public LinearSlidesEncoder(DcMotor linearSlideMotor, Servo claw, LinearOpMode linearOp){
        this.linearSlideMotor = linearSlideMotor;
        this.claw = claw;
        this.linearOp = linearOp;
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setTargetPosition(0);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public void moveToPosition(LinearPosition pos, double power) {
        currentLinearPosition = pos;
        linearSlideMotor.setTargetPosition(pos.ticks);
        linearSlideMotor.setPower(power);
    }

    public void openClaw(){
        changeClawPos(1);
        isOpen = true;
    }
    public void closeClaw(){
        changeClawPos(0.15);
        isOpen = false;
    }
    public void changeClawPos(double position){
        this.claw.setPosition(Range.clip(position, MIN_POS, MAX_POS));
    }

    public void analogMoveSlide(float magnitude)
    {
        linearSlideMotor.setTargetPosition((int) (linearSlideMotor.getCurrentPosition() + Math.floor(magnitude * 160)));
        linearSlideMotor.setPower(magnitude);
    }

    public int getCurrentPosition(){
        return linearSlideMotor.getCurrentPosition();
    }
    public void sleep(long milli){linearOp.sleep(milli);}
}