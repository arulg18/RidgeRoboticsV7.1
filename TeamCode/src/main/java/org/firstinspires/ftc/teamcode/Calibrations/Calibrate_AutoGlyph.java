package org.firstinspires.ftc.teamcode.Calibrations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Auto Glyph Calibration", group = "Test")

public class Calibrate_AutoGlyph extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.all);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            jewelFlick.setPosition(0);
            sweepServo(RGrabber, 1, 0.01, 50);
            jewelFlick.setPosition(START_POSITION_FLICK);
            jewelDown.setPosition(0);
            sweepServo(LGrabber, 1, 0.01, 50);
            jewelDown.setPosition(START_POSITION_DOWN);
            break;


        }
    }

}
