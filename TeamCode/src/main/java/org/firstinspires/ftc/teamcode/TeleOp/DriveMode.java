package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/15/17.
 */
@TeleOp(name="Tele-OP Regular", group = "Main")

public class DriveMode extends Central {

    public static float yAxis1;
    public static float xAxis1;

    public static float yAxis2;
    public static float xAxis2;

    public static float fb;
    public static float rl;

    public static double diagonalSpeed;

    public static boolean rightStickButtonPressed;
    public static boolean leftStickButtonPressed;

    public static boolean x = true;
    public static boolean bPrevState = false;
    public static boolean bCurrState = false;
//UNCOMMENT CRAWL MODE ACTIVATED TLEMETRY
    @Override
    public void runOpMode() throws InterruptedException {
        CentralClass(setupType.teleop);
        waitForStart();

        while (opModeIsActive()) {
            //ADD CRAW

            yAxis1 = -gamepad1.left_stick_y; // Main Directions y-axis
            xAxis1 = gamepad1.left_stick_x;  // Main Directions x-axis

            yAxis2 = -gamepad1.right_stick_y; // Diagonal Directions y-axis
            xAxis2 = gamepad1.right_stick_x;  // Diagonal Directions x-axis

            fb = Math.abs(yAxis1);
            rl = Math.abs(xAxis1);

            rightStickButtonPressed = gamepad1.right_stick_button;  // Button Directions y-axis
            leftStickButtonPressed = gamepad1.left_stick_button;   // Button Directions x-axis

            diagonalSpeed = Math.sqrt(Math.pow(yAxis2, 2) + Math.pow(xAxis2, 2));

            // clip the right/left values so that the values never exceed +/- 1
            yAxis1 = Range.clip(yAxis1, -1, 1);
            xAxis1 = Range.clip(xAxis1, -1, 1);

            yAxis2 = Range.clip(yAxis2, -1, 1);
            xAxis2 = Range.clip(xAxis2, -1, 1);



            if (gamepad1.x){
                balancer(startX, startY, gamepad1, 0.2);
            }

            bCurrState = gamepad1.a;

            if ((bCurrState == true) && (bCurrState != bPrevState)) {
                x = !x;

            }
            bPrevState = bCurrState;


            if (x) {
                //telemetry.addLine("Regular Mode Activated");
                //telemetry.update();
                if (leftStickButtonPressed) {
                    // CLOCKWISE
                    driveTrainMovement(ROTATION_SPEED, movements.cw);
                } else if (rightStickButtonPressed) {
                    // COUNTERCLOCKWISE
                    driveTrainMovement(ROTATION_SPEED, movements.ccw);
                } else if (gamepad1.dpad_up) {
                    driveTrainMovement(D_PAD_SPEED, movements.forward);
                    //FORWARD
                } else if (gamepad1.dpad_down) {
                    driveTrainMovement(D_PAD_SPEED, movements.backward);
                    //BACKWARD
                } else if (gamepad1.dpad_right) {
                    driveTrainMovement(D_PAD_SPEED, movements.right);
                    //RIGHT
                } else if (gamepad1.dpad_left) {
                    driveTrainMovement(D_PAD_SPEED, movements.left);
                    //LEFT
                } else if (Math.pow(xAxis1, 2) + Math.pow(yAxis1, 2) >= Math.pow(DEAD_ZONE_SIZE, 2) && !leftStickButtonPressed) { //MAIN DIRECTIONS

                    if (yAxis1 >= Math.abs(xAxis1)) {
                        driveTrainMovement(fb, movements.forward);
                        //FORWARD
                    } else if (yAxis1 <= -Math.abs(xAxis1)) {
                        driveTrainMovement(fb, movements.backward);
                        //BACKWARD
                    } else if (Math.abs(yAxis1) < xAxis1) {
                        driveTrainMovement(rl, movements.right);
                        //RIGHT
                    } else if (-Math.abs(yAxis1) > xAxis1) {
                        driveTrainMovement(rl, movements.left);
                        //LEFT
                    }
                } else if (Math.pow(xAxis2, 2) + Math.pow(yAxis2, 2) >= Math.pow(DEAD_ZONE_SIZE, 2) && !rightStickButtonPressed) {    //DIAGONAL

                    if (yAxis2 >= 0 && xAxis2 >= 0) {
                        driveTrainMovement(diagonalSpeed, movements.tr);
                        //TOP RIGHT
                    } else if (yAxis2 >= 0 && xAxis2 < 0) {
                        driveTrainMovement(diagonalSpeed, movements.tl);
                        //TOP LEFT
                    } else if (yAxis2 < 0 && xAxis2 >= 0) {
                        driveTrainMovement(diagonalSpeed, movements.br);
                        //BOTTOM RIGHT
                    } else if (yAxis2 < 0 && xAxis2 < 0) {
                        driveTrainMovement(diagonalSpeed, movements.bl);
                        //BOTTOM LEFT
                    }
                } else {
                    stopDrivetrain();
                }
            }
            else {
                ///telemetry.addLine("Crawl Mode Activated");
                //telemetry.update();
                if (leftStickButtonPressed) {
                    // CLOCKWISE
                    driveTrainMovement(CRAWL_SPEED, movements.cw);
                } else if (rightStickButtonPressed) {
                    // COUNTERCLOCKWISE
                    driveTrainMovement(CRAWL_SPEED, movements.ccw);
                } else if (gamepad1.dpad_up) {
                    driveTrainMovement(D_PAD_SPEED, movements.forward);
                    //FORWARD
                } else if (gamepad1.dpad_down) {
                    driveTrainMovement(D_PAD_SPEED, movements.backward);
                    //BACKWARD
                } else if (gamepad1.dpad_right) {
                    driveTrainMovement(D_PAD_SPEED, movements.right);
                    //RIGHT
                } else if (gamepad1.dpad_left) {
                    driveTrainMovement(D_PAD_SPEED, movements.left);
                    //LEFT
                } else if (Math.pow(xAxis1, 2) + Math.pow(yAxis1, 2) >= Math.pow(DEAD_ZONE_SIZE, 2) && !leftStickButtonPressed) { //MAIN DIRECTIONS

                    if (yAxis1 >= Math.abs(xAxis1)) {
                        driveTrainMovement(CRAWL_SPEED, movements.forward);
                        //FORWARD
                    } else if (yAxis1 <= -Math.abs(xAxis1)) {
                        driveTrainMovement(CRAWL_SPEED, movements.backward);
                        //BACKWARD
                    } else if (Math.abs(yAxis1) < xAxis1) {
                        driveTrainMovement(CRAWL_SPEED, movements.right);
                        //RIGHT
                    } else if (-Math.abs(yAxis1) > xAxis1) {
                        driveTrainMovement(CRAWL_SPEED, movements.left);
                        //LEFT
                    }
                } else if (Math.pow(xAxis2, 2) + Math.pow(yAxis2, 2) >= Math.pow(DEAD_ZONE_SIZE, 2) && !rightStickButtonPressed) {    //DIAGONAL

                    if (yAxis2 >= 0 && xAxis2 >= 0) {
                        driveTrainMovement(CRAWL_SPEED, movements.tr);
                        //TOP RIGHT
                    } else if (yAxis2 >= 0 && xAxis2 < 0) {
                        driveTrainMovement(CRAWL_SPEED, movements.tl);
                        //TOP LEFT
                    } else if (yAxis2 < 0 && xAxis2 >= 0) {
                        driveTrainMovement(CRAWL_SPEED, movements.br);
                        //BOTTOM RIGHT
                    } else if (yAxis2 < 0 && xAxis2 < 0) {
                        driveTrainMovement(CRAWL_SPEED, movements.bl);
                        //BOTTOM LEFT
                    }
                } else {
                    stopDrivetrain();
                }

            }


            if (gamepad2.x){
                pullServo.setPosition(pullServo.getPosition() + 0.04);
                sleep(30);
            }
            else if (gamepad2.b){
                pullServo.setPosition(pullServo.getPosition() - 0.04);
                sleep(30);
            }

            if (gamepad2.y){
                rightTread.setPower(0.6);
                leftTread.setPower(0.6);
                sleep(30);
            }
            else if (gamepad2.right_bumper){
                rightTread.setPower(1);
                leftTread.setPower(1);
                sleep(30);
            }
            else if (gamepad2.a){
                rightTread.setPower(-0.6);
                leftTread.setPower(-0.6);
                sleep(30);
            }
            else {
                rightTread.setPower(0);
                leftTread.setPower(0);
            }

            telemetry.addData("IMU Angular Orientation: ", imu.getAngularOrientation());


            telemetry.update();
            /*
            if (gamepad2.dpad_up){
                relicMotorIn.setPower(-0.6);
                relicMotorOut.setPower(-0.6);
                telemetry.addLine("Moving Up");
                telemetry.update();
                sleep(30);
            }
            else if (gamepad2.dpad_down){
                relicMotorIn.setPower(0.6);
                relicMotorOut.setPower(0.6);
                sleep(30);
                telemetry.addLine("Moving Down");
                telemetry.update();
            }
            else {
                relicMotorIn.setPower(0);
                relicMotorOut.setPower(0);
            }

            if (gamepad2.left_trigger > 0.25){
                relicMotorOut.setPower(-0.6);
                telemetry.addLine("Moving Up");
                telemetry.update();
                sleep(30);
            }
            else if (gamepad2.right_trigger > 0.25){
                relicMotorOut.setPower(0.6);
                sleep(30);
                telemetry.addLine("Moving Down");
                telemetry.update();
            }
            else {
                relicMotorOut.setPower(0);
            }
            */
        }
//        initialPositionFlicker(0);

    }

}
