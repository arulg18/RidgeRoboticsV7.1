package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Grab Test", group = "Test")

public class Test_GrabRelic extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.relic);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Position: ", angleServo.getPosition());
            telemetry.update();
            angleServo.setPosition(0.2);
            Claw.setPosition(0.2);
            telemetry.addData("Position: ", angleServo.getPosition());
            telemetry.update();
            sleep(2000);
            angleServo.setPosition(0.4);
            Claw.setPosition(0.4);
            telemetry.addData("Position: ", angleServo.getPosition());
            telemetry.update();
            sleep(2000);
            angleServo.setPosition(0.6);
            Claw.setPosition(0.6);
            telemetry.addData("Position: ", angleServo.getPosition());
            telemetry.update();
            sleep(2000);
            angleServo.setPosition(0.8);
            Claw.setPosition(0.8);
            telemetry.addData("Position: ", angleServo.getPosition());
            telemetry.update();
            sleep(2000);
            angleServo.setPosition(1);
            Claw.setPosition(1);
            telemetry.addData("Position: ", angleServo.getPosition());
            telemetry.update();
            sleep(2000);
            break;
        }
    }

}
