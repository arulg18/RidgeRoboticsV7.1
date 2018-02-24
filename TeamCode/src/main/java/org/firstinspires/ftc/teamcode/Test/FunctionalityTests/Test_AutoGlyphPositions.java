package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Auto Glyph Position Test", group = "Test")

public class Test_AutoGlyphPositions extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.all);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            RGrabber.setPosition(RGRAB_POSITION);
            LGrabber.setPosition(LGRAB_POSITION);
            sleep(3000);
            break;

        }
    }

}
