package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.enums.StartPosition;


@Autonomous(name="Front Auto", group="Autonomous")

public class FrontAutonomous extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        FirstCenterStage.run(this, StartPosition.FRONT);
    }
}