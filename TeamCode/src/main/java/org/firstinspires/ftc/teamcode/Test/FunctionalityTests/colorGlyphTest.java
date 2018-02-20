package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Test Color Glyph", group = "Test")

public class colorGlyphTest extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        glyphColorSensor = colorSensor(glyphColorSensor, hardwareMap, glyphColorSensorS, true);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Color Value: ", jewelSensor.argb());
            telemetry.update();


        }
    }

}
