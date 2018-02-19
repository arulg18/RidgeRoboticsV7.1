package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Tread Test", group = "Test")

public class Test_Treads extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        rightTread = motor(rightTread, hardwareMap, rightTreadS, DcMotorSimple.Direction.FORWARD);
        leftTread = motor(leftTread, hardwareMap, leftTreadS, DcMotorSimple.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addLine("Speed 0.2");
            telemetry.update();
            powerMotors(0.2, 5000, rightTread, leftTread);
            sleep(1000);
            telemetry.addLine("Speed 0.4");
            telemetry.update();
            powerMotors(0.4, 5000, rightTread, leftTread);
            sleep(1000);
            telemetry.addLine("Speed 0.6");
            telemetry.update();
            powerMotors(0.6, 5000, rightTread, leftTread);
            sleep(1000);
            telemetry.addLine("Speed 0.8");
            telemetry.update();
            powerMotors(0.8, 5000, rightTread, leftTread);
            sleep(1000);
            telemetry.addLine("Speed 1.0");
            telemetry.update();
            powerMotors(1.0, 5000, rightTread, leftTread);
            sleep(1000);
            break;

        }
    }

}
