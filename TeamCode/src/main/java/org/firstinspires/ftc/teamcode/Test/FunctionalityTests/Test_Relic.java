package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Relic Test 1", group = "Test")

public class Test_Relic extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.relic);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            encoderMovement(0.2, 4, 5, 1000, movements.relicOut, relicMotorIn);
            encoderMovement(0.2, 4, 5, 1000, movements.relicIn, relicMotorIn);

            break;
        }
    }

}
