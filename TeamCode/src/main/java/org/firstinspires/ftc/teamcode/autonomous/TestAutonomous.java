package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="TestAutonomous", group="Autonomous")

public class TestAutonomous extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        FirstCenterStage.run(this);
    }
}