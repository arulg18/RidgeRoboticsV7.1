package org.firstinspires.ftc.teamcode.Calibrations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Jewel Get Position", group = "Test")

public class Get_Jewel_Position extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.jewel);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            jewelDown.setPosition(LOW_POSITION_DOWN);
            telemetry.addLine("JEWEL LOW DOWN POSITION");
            telemetry.addData("Jewel Down: ", jewelDown.getPosition());
            telemetry.addData("Jewel Flick: ", jewelFlick.getPosition());
            telemetry.update();
            sleep(2000);
            jewelFlick.setPosition(RIGHT_POSITION_FLICK);
            telemetry.addLine("JEWEL RIGHT FLICK POSITION");
            telemetry.addData("Jewel Down: ", jewelDown.getPosition());
            telemetry.addData("Jewel Flick: ", jewelFlick.getPosition());
            telemetry.update();
            sleep(2000);
            jewelFlick.setPosition(LEFT_POSITION_FLICK);
            telemetry.addLine("JEWEL LEFT FLICK POSITION");
            telemetry.addData("Jewel Down: ", jewelDown.getPosition());
            telemetry.addData("Jewel Flick: ", jewelFlick.getPosition());
            telemetry.update();
            sleep(2000);
            sleep(10000);
            break;


        }
    }

}
