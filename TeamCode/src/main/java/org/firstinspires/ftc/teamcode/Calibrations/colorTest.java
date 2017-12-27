package org.firstinspires.ftc.teamcode.Calibrations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Test Color", group = "Test")

public class colorTest extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        jewelSensor = colorSensor(jewelSensor, hardwareMap, jewelSensorS, true);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Red: ", jewelSensor.red());
            telemetry.addData("Blue: ", jewelSensor.blue());
            telemetry.addData("Green: ", jewelSensor.green());
            telemetry.update();
            //

        }
    }

}
