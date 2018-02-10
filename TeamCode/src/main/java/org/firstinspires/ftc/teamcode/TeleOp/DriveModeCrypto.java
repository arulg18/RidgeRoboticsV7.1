package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Central;

import java.util.ArrayList;
import java.util.Arrays;


/**
 * Created by arulgupta on 12/15/17.
 */
@TeleOp(name="Tele-OP Crypto", group = "Main")

public class DriveModeCrypto extends Central {



    public final int BROWN  = 2;
    public final int GRAY   = 1;

    public enum robotState{
        pickup, picked, placed;
    }

    public static final int[][] frog1 = {{2,1,2}, {1,2,1}, {2,1,2}, {1,2,1}};
    public static final int[][] frog2 = {{1,2,1}, {2,1,2}, {1,2,1}, {2,1,2}};

    public static final int[][] bird1 = {{1,2,1}, {2,1,2}, {2,1,2}, {1,2,1}};
    public static final int[][] bird2 = {{2,1,2}, {1,2,1}, {1,2,1}, {2,1,2}};

    public static final int[][] snake1 = {{1, 1, 2}, {1, 2, 2}, {2, 2, 1}, {2, 1, 1}};
    public static final int[][] snake2 = {{2, 2, 1}, {2, 1, 1}, {1, 1, 2}, {1, 2, 2}};

    public static int[][][] p = {frog1, frog2, bird1, bird2, snake1, snake2};
    public static ArrayList<int[][]> possibles = new ArrayList<>(Arrays.asList(p));

    public static int[][] current = new int[4][3];
    public static int[] emptyPositions = {0, 0, 0};
    public static int[] emptyColors = {0, 0};


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
        for (int[] row: current){
            Arrays.fill(row, 0);
        }
        robotState currentState = robotState.pickup;
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
                telemetry.addData("IMU Angular Orientation: ", imu.getAngularOrientation());
                telemetry.update();
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

            currentState = state(currentState);

        }
//        initialPositionFlicker(0);

    }


    public robotState state(robotState stateN) {
        displayCryptobox();
        switch (stateN) {
            case placed:
                telemetry.addLine("dpad position left = left, center = up, right = right: ");
                telemetry.update();
                int startP = 1;
                do {
                    if (gamepad2.dpad_left) {
                        startP = 1;
                        break;
                    } else if (gamepad2.dpad_up) {
                        startP = 2;
                        break;
                    } else if (gamepad2.dpad_right) {
                        startP = 3;
                        break;
                    }
                }
                while (opModeIsActive());

                telemetry.addLine("Glyph color, gray = left, brown = right");
                telemetry.update();
                int color = 1;
                do {
                    if (gamepad2.dpad_left) {
                        color = 1;
                        break;
                    } else if (gamepad2.dpad_right) {
                        color = 2;
                        break;
                    }
                }
                while (opModeIsActive());

                current[0][startP - 1] = color;
                return robotState.pickup;
            case pickup:
                int[] colors = predictionCount();

                telemetry.addLine("GRAY BROWN");
                telemetry.addLine(" " + colors[0] + "     " + colors[1] + "  ");
                telemetry.update();
                return robotState.picked;
            case picked:
                telemetry.addLine("What did you pick? Glyph color, gray = 1, brown = 2");
                telemetry.update();
                int chosen = 1;
                do {
                    if (gamepad2.dpad_left) {
                        chosen = 1;
                        break;
                    } else if (gamepad2.dpad_right) {
                        chosen = 2;
                        break;
                    }
                }
                while (opModeIsActive());
                telemetry.addLine("LEFT CENTER RIGHT");
                int[] positions = reactionCount(chosen);
                telemetry.addLine("  " + positions[0] + "     " + positions[1] + "     " + positions[2] + " ");
                telemetry.update();

                return robotState.placed;

        }
        return stateN;
    }




    public static void printState(){
        System.out.printf("Number of possibles: %d%n", possibles.size());
        System.out.println("Current Row: " + currentRowToString());
    }
    public static void displayCryptobox(){
        for (int i = current.length - 1; i >= 0; i--){
            String line = "";
            String p1 = (current[i][0] == 0 ? "_" : (current[i][0] == 1 ? "G" : "B"));
            String p2 = (current[i][1] == 0 ? "_" : (current[i][1] == 1 ? "G" : "B"));
            String p3 = (current[i][2] == 0 ? "_" : (current[i][2] == 1 ? "G" : "B"));

            System.out.println(p1 + "|" + p2 + "|" + p3);
        }
    }
    public static void displayPossibles(){
        for (int i = 0; i < possibles.size(); i++){
            System.out.println("Cryptobox " + i);
            for (int[] x : possibles.get(i)) {
                System.out.println(Arrays.toString(x));
            }
        }
    }
    public static String prediction(){
        int[] count = predictionCount();

        String finalMessage = "";
        finalMessage = Arrays.toString(count);
        return finalMessage;



    }
    public static String reaction(int color){
        int[] count = reactionCount(color);

        String finalMessage = "";
        finalMessage = Arrays.toString(count);
        return finalMessage;


    }

    public static int[] predictionCount(){
        int[] color = new int[2];
        int currentRow = currentRow();
        ArrayList<int[][]> remove = new ArrayList<>();
        for (int x = 0; x < possibles.size(); x++){

            int[] colorsBox = cryptoboxColor(possibles.get(x), currentRow);
            if (colorsBox[0] == 0 && colorsBox[1] == 0){
                remove.add(possibles.get(x));
            }else {
                color[0] += colorsBox[0];
                color[1] += colorsBox[1];
            }
        }

        possibles.removeAll(remove);


        return color;
    }

    public static int[] cryptoboxColor(int[][] box, int row){
        int[] colors = {0,0};
        for (int i = 0; i < row; i++){
            if (!Arrays.equals(box[i], current[i])){
                return colors;
            }
        }
        for (int i = 0; i < box[row].length; i++){

            if (box[row][i] != current[row][i] && current[row][i] != 0){
                colors[0] = 0;
                colors[1] = 0;
                return colors;
            }
            else if (box[row][i] != current[row][i]){
                colors[(box[row][i] == 1 ? 0 : 1)]++;
            }
        }
        return colors;

    }
    public static int[] reactionCount(int chosen){
        int[] positions = new int[3];
        int currentRow = currentRow();
        ArrayList<Integer> remove = new ArrayList<>();
        for (int x = 0; x < possibles.size(); x++){

            int[] positionsBox = cryptoboxPosition(possibles.get(x), currentRow, chosen);
            if (positionsBox[0] == 0 && positionsBox[1] == 0 && positionsBox[2] == 0){
                remove.add(x);
            }else {
                positions[0] += positionsBox[0];
                positions[1] += positionsBox[1];
                positions[2] += positionsBox[2];
            }

        }
        for (int i : remove){
            possibles.removeAll(remove);

        }


        return positions;
    }
    public static int[] cryptoboxPosition(int[][] box, int row, int color){
        int[] position = {0,0,0};

        for (int i = 0; i < box[row].length; i++){
            if (box[row][i] == color && current[row][i] == 0){
                position[i]++;
            }

        }

        return position;
    }

    public static int currentRow(){
        for (int i = 0; i < current.length; i++){
            for (int x: current[i]){
                if (x == 0){
                    return i;
                }
            }
        }
        return -1;

    }

    public static String currentRowToString(){
        int row = currentRow();
        switch (row){
            case 0:
                return "1st";

            case 1:
                return "2nd";

            case 2:
                return "3rd";

            case 3:
                return "4th";
        }
        return "";
    }


    public static String processReaction(int[] count){



        int highest = Math.max(count[0], Math.max(count[1], count[2]));


        int numberOf = 0;
        int greaters = 0;
        for (int i : count){
            if (highest == i){
                numberOf++;
            }
            if (i > 0){
                greaters++;
            }
        }

        if (greaters == 0){
            return "NO BOX POSSIBLE";
        }
        else if (greaters == 1) {
            if (count[0] == highest && count[1] == 0 && count[2] == 0) {
                return "ONLY left";
            } else if (count[1] == highest && count[0] == 0 && count[2] == 0) {
                return "ONLY center";
            } else if (count[2] == highest && count[0] == 0 && count[1] == 0) {
                return "ONLY right";
            }
        }

        else if (greaters == 2 && numberOf == 2){
            if (count[0] == count[2]){
                return "EITHER left or right";
            }
            else if (count[1] == count[2]){
                return "EITHER center or right";
            }
            else {
                return "EITHER left or center";
            }
        }
        else if (greaters == 2){
            if (count[0] == highest){
                return "BEST left";
            }
            else if (count[1] == highest){
                return "BEST center";
            }
            else if (count[2] == highest){
                return "BEST right";
            }
        }
        else if (numberOf == 3){
            return "ANY POSITION";
        }

        return "Nothing";

    }



}
