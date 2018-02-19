/*package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/15/17.
 */
//@TeleOp(name="Tele-OP Relic", group = "Main")
/*
public class DriveModeRelic extends Central {

    @Override
    public void runOpMode() throws InterruptedException {
        CentralClass(setupType.relic);
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Angle Servo Speed: ", angleServo.getPosition());

            if (gamepad2.dpad_right){
                relicMotorIn.setPower(-1);
            }
            else if (gamepad2.dpad_left){
                relicMotorIn.setPower(1);

            }
            else {
                relicMotorIn.setPower(0);
            }

            if (gamepad2.dpad_up){
                angleServo.setPower(0.3);
                telemetry.addLine("Angle Up");
                telemetry.update();
            }
            else if (gamepad2.dpad_down){
                angleServo.setPower(-0.3);
                telemetry.addLine("Angle Down");
                telemetry.update();
            }
            else {
                angleServo.setPower(0);
                telemetry.addLine("Angle None");
                telemetry.update();
            }

            if (gamepad2.a){
                Claw.setPosition(Claw.getPosition() + 0.04);
                sleep(30);
                telemetry.addLine("Claw Up");
                telemetry.update();
            }
            else if (gamepad2.y){
                Claw.setPosition(Claw.getPosition() - 0.04);
                sleep(30);
                telemetry.addLine("Claw Down");
                telemetry.update();
            }else {
                telemetry.addLine("None");
                telemetry.update();
            }


        }
    }

}
*/