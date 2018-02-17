package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/15/17.
 */
@TeleOp(name="Tele-OP Relic", group = "Main")

public class DriveModeRelic extends Central {

    @Override
    public void runOpMode() throws InterruptedException {
        CentralClass(setupType.relic);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.dpad_right){
                relicMotorIn.setPower(-1);
                sleep(30);
            }
            else if (gamepad2.dpad_left){
                relicMotorIn.setPower(1);
                sleep(30);

            }
            else {
                relicMotorIn.setPower(0);
            }

        }
    }

}
