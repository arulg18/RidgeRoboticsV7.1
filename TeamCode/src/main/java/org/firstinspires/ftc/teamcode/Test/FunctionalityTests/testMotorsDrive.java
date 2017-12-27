package org.firstinspires.ftc.teamcode.Test.SystemTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Motors Drive Test", group = "Test")

public class testMotorsDrive extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.drive);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            motorFR.setPower(0.3);
            telemetry.addData("MotorFR speed: ", motorFR.getPower());
            telemetry.update();
            sleep(3000);

            motorFL.setPower(0.3);
            telemetry.addData("MotorFL speed: ", motorFL.getPower());
            telemetry.update();
            sleep(3000);

            motorBR.setPower(0.3);
            telemetry.addData("MotorBR speed: ", motorBR.getPower());
            telemetry.update();
            sleep(3000);

            motorBL.setPower(0.3);
            telemetry.addData("MotorBL speed: ", motorBL.getPower());
            telemetry.update();
            sleep(3000);

            break;

        }
    }

}
