package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Changer Test", group = "Test")
public class Test_Changer extends Central {

    public ElapsedTime runtime = new ElapsedTime();
    public int tester = 1;

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        super.setTestee(tester);

        CentralClass(setupType.all);
        waitForStart();

        runtime.reset();

    }

}
