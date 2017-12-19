package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;



/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Jewel Test", group = "Test")

public class Test_Jewel extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.all);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            flick(team.red1);
            telemetry.addLine("Running as Red 1");
            flick(team.blue1);
            telemetry.addLine("Running as Red 1");

            break;

        }
    }

}
