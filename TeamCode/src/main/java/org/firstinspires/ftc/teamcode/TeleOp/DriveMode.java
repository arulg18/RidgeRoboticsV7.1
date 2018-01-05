package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/15/17.
 */
@TeleOp(name="Tele-OP", group = "Main")

public class DriveMode extends Central {


    @Override
    public void runOpMode() throws InterruptedException {
        CentralClass(setupType.teleop);
        waitForStart();

        while (opModeIsActive()) {
            //ADD CRAW
            //tipcorrect();
            float yAxis1 = -gamepad1.left_stick_y; // Main Directions y-axis
            float xAxis1 = gamepad1.left_stick_x;  // Main Directions x-axis

            float yAxis2 = -gamepad1.right_stick_y; // Diagonal Directions y-axis
            float xAxis2 = gamepad1.right_stick_x;  // Diagonal Directions x-axis

            float fb = Math.abs(yAxis1);
            float rl = Math.abs(xAxis1);

            boolean rightStickButtonPressed = gamepad1.right_stick_button;  // Button Directions y-axis
            boolean leftStickButtonPressed = gamepad1.left_stick_button;   // Button Directions x-axis

            double diagonalSpeed = Math.sqrt(Math.pow(yAxis2, 2) + Math.pow(xAxis2, 2));

            boolean crawlModeIsOn = false;

            // clip the right/left values so that the values never exceed +/- 1
            yAxis1 = Range.clip(yAxis1, -1, 1);
            xAxis1 = Range.clip(xAxis1, -1, 1);

            yAxis2 = Range.clip(yAxis2, -1, 1);
            xAxis2 = Range.clip(xAxis2, -1, 1);

            if (gamepad1.right_bumper) {
                crawlModeIsOn = !crawlModeIsOn;
            }
            if (crawlModeIsOn) {
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

        }
    }
}
