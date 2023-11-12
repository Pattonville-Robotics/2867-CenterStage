package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.enums.StartPosition;


@Autonomous(name="Blue Backstage Auto", group="Autonomous")

public class BlueBackAutonomous extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        FirstCenterStage.run(this, StartPosition.BACKSTAGE_BLUE);
    }
}