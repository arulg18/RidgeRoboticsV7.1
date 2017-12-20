package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;



/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Mecanum Test", group = "Test")

public class Test_Mecanum extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.drive);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            for(movements movement: allMovements){
                driveTrainEncoderMovement(0.7, 12, 10, 3000, movement);
            }
            break;

        }
    }

}
