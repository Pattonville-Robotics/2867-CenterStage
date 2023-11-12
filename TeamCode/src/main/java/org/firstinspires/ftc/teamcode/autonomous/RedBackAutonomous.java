package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.enums.StartPosition;


@Autonomous(name="Red Backstage Auto", group="Autonomous")

public class RedBackAutonomous extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        FirstCenterStage.run(this, StartPosition.BACKSTAGE_RED);
    }
}