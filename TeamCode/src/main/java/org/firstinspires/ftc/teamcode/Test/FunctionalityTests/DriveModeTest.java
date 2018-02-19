package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/15/17.
 */
@TeleOp(name="Tele-OP testcode", group = "Main")

public class DriveModeTest extends Central {


    @Override
    public void runOpMode() throws InterruptedException {
        CentralClass(setupType.teleop);
        waitForStart();

        while (opModeIsActive()) {
            //ADD CRAW

            float yAxis1 = -gamepad1.left_stick_y; // Main Directions y-axis
            float xAxis1 = gamepad1.left_stick_x;  // Main Directions x-axis

            float yAxis2 = -gamepad1.right_stick_y; // Diagonal Directions y-axis
            float xAxis2 = gamepad1.right_stick_x;  // Diagonal Directions x-axis

            float fb = Math.abs(yAxis1);
            float rl = Math.abs(xAxis1);

            boolean rightStickButtonPressed = gamepad1.right_stick_button;  // Button Directions y-axis
            boolean leftStickButtonPressed = gamepad1.left_stick_button;   // Button Directions x-axis

            double diagonalSpeed = Math.sqrt(Math.pow(yAxis2, 2) + Math.pow(xAxis2, 2));

            // clip the right/left values so that the values never exceed +/- 1
            yAxis1 = Range.clip(yAxis1, -1, 1);
            xAxis1 = Range.clip(xAxis1, -1, 1);

            yAxis2 = Range.clip(yAxis2, -1, 1);
            xAxis2 = Range.clip(xAxis2, -1, 1);



            if(leftStickButtonPressed){
                // CLOCKWISE
                telemetry.addLine("Moving Clockwise");
                telemetry.addData("X: ", xAxis1);
                telemetry.addData("Y: ", yAxis1);
                telemetry.update();

            }
            else if (rightStickButtonPressed){
                // COUNTERCLOCKWISE
                telemetry.addLine("Moving Counterclockwise");
                telemetry.addData("X: ", xAxis1);
                telemetry.addData("Y: ", yAxis1);
                telemetry.update();

            }
            else if (gamepad1.dpad_up){
                telemetry.addLine("Moving Forward");
                telemetry.addData("X: ", xAxis1);
                telemetry.addData("Y: ", yAxis1);
                telemetry.update();

                //FORWARD
            }
            else if (gamepad1.dpad_down){
                telemetry.addLine("Moving Downward");
                telemetry.addData("X: ", xAxis1);
                telemetry.addData("Y: ", yAxis1);
                telemetry.update();

                //BACKWARD
            }
            else if (gamepad1.dpad_right){
                telemetry.addLine("Moving Right");
                telemetry.addData("X: ", xAxis1);
                telemetry.addData("Y: ", yAxis1);
                telemetry.update();

                //RIGHT
            }
            else if (gamepad1.dpad_left){
                telemetry.addLine("Moving Left");
                telemetry.addData("X: ", xAxis1);
                telemetry.addData("Y: ", yAxis1);
                telemetry.update();

                //LEFT
            }
            else if(Math.pow(xAxis1, 2) + Math.pow(yAxis1, 2) >= Math.pow(DEAD_ZONE_SIZE, 2) && !leftStickButtonPressed){ //MAIN DIRECTIONS

                if (yAxis1 >= Math.abs(xAxis1)){
                    telemetry.addLine("Moving Forward");
                    telemetry.addData("X: ", xAxis1);
                    telemetry.addData("Y: ", yAxis1);
                    telemetry.update();

                    //FORWARD
                }
                else if (yAxis1 <= -Math.abs(xAxis1)){
                    telemetry.addLine("Moving Backward");
                    telemetry.addData("X: ", xAxis1);
                    telemetry.addData("Y: ", yAxis1);
                    telemetry.update();

                    //BACKWARD
                }
                else if (Math.abs(yAxis1) < xAxis1){
                    telemetry.addLine("Moving Right");
                    telemetry.addData("X: ", xAxis1);
                    telemetry.addData("Y: ", yAxis1);
                    telemetry.update();

                    //RIGHT
                }
                else if (-Math.abs(yAxis1) > xAxis1){
                    telemetry.addLine("Moving Left");
                    telemetry.addData("X: ", xAxis1);
                    telemetry.addData("Y: ", yAxis1);
                    telemetry.update();

                    //LEFT
                }
            }
            else if (Math.pow(xAxis2, 2) + Math.pow(yAxis2, 2) >= Math.pow(DEAD_ZONE_SIZE, 2) && !rightStickButtonPressed) {    //DIAGONAL

                if (yAxis2 >= 0 && xAxis2 >= 0){
                    telemetry.addLine("Moving Top Right");
                    telemetry.addData("X: ", xAxis2);
                    telemetry.addData("Y: ", yAxis2);
                    telemetry.update();

                    //TOP RIGHT
                }
                else if (yAxis2 >= 0 && xAxis2 < 0){
                    telemetry.addLine("Moving Top Left");
                    telemetry.addData("X: ", xAxis2);
                    telemetry.addData("Y: ", yAxis2);
                    telemetry.update();

                    //TOP LEFT
                }
                else if (yAxis2 < 0 && xAxis2 >= 0){
                    telemetry.addLine("Moving Bottom Right");
                    telemetry.addData("X: ", xAxis2);
                    telemetry.addData("Y: ", yAxis2);
                    telemetry.update();

                    //BOTTOM RIGHT
                }
                else if (yAxis2 < 0 && xAxis2 < 0){
                    telemetry.addLine("Moving Bottom Left");
                    telemetry.addData("X: ", xAxis2);
                    telemetry.addData("Y: ", yAxis2);
                    telemetry.update();

                    //BOTTOM LEFT
                }
            }
            else {
                telemetry.addLine("Stopped.");
                telemetry.addData("X: ", xAxis1);
                telemetry.addData("Y: ", yAxis1);
                telemetry.update();
                stopDrivetrain();
            }




        }
    }
}
