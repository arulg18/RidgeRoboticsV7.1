package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Central;

/**
 * Created by adith on 2/17/2018.
 */

@TeleOp(name="FakeBot Test", group = "Main")

public class OurRealNotFakeOriginalCompetitionRobot  extends Central {
    DcMotor rightMotor;
    DcMotor leftMotor;

    public static final String rightMotorS = "rightMotor";
    public static final String leftMotorS= "leftMotor";
    public DcMotor motor(DcMotor motor, HardwareMap hardwareMap, String name, DcMotor.Direction direction) throws InterruptedException{
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        return motor;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        rightMotor = motor(rightMotor, hardwareMap, rightMotorS, DcMotorSimple.Direction.FORWARD);
        leftMotor = motor(leftMotor, hardwareMap, leftMotorS, DcMotorSimple.Direction.FORWARD);
        waitForStart();
        while(opModeIsActive()) {
            float leftPower = gamepad1.left_stick_y;
            float rightPower = gamepad1.right_stick_y;
            if (rightPower > -0.1 && rightPower < 0.1){
                rightPower = 0;
            }

            if (leftPower > -0.1 && leftPower < 0.1){
                leftPower = 0;
            }
            rightMotor.setPower(-rightPower);
            leftMotor.setPower(leftPower);
        }
    }

}