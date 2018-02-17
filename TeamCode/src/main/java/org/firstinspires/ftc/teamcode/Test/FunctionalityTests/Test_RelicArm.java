package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Relic Test", group = "Test")

public class Test_RelicArm extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.relic);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            angleServo.setPosition(0.4);
            Claw.setPosition(.45);
            angleServo.setPosition(0);
            break;
        }
    }

}
