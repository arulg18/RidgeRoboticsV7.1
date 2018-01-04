package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Rotation Test", group = "Test")

public class RotationTest extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        super.setRuntime(runtime);
        waitForStart();
        turn(90, turnside.cw, 1);
        telemetry.addLine("90 degrees, Speed 1, clockwise, center axis");
        telemetry.update();
        turn(90, turnside.ccw, 0.5);
        telemetry.addLine("90 degrees, Speed 0.5, counterclockwise, center axis");
        telemetry.update();
        turn(90, turnside.ccw, 0.5, axis.front);
        telemetry.addLine("90 degrees, Speed 0.5, counterclockwise, front axis");
        telemetry.update();
        turn(45, turnside.cw, 1, axis.back);
        telemetry.addLine("45 degrees, Speed 1, counterclockwise, back axis");
        telemetry.update();
    }
}
