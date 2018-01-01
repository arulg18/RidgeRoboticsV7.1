package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

//IMMEDIATES TO MAKE THINGS RUN PROPERLY
/*
* FINISH SETUP FUNCTION
*
 */
//THINGS TO ADD LIST BELOW
/*
* Jewel flicker if too close moves back a little
* ADD CRAWL MODE FOR GAMEPAD
* GYROSCOPE FOR BALANCE WITH IMU
* GLYPH SYSTEM SETUP
* */

public class Central extends LinearOpMode{
    //-------------------SUBCLASS VARIABLES----------------------
    private ElapsedTime runtime = new ElapsedTime();

    //--------------------------CONSTANTS----------------------------

            //--------------------------ENCODERS---------------------
                public static final double COUNTS_PER_MOTOR_NEVEREST = 1680;
                public static final double COUNTS_PER_MOTOR_TETRIX = 1440;
                public static final double DRIVE_GEAR_REDUCTION = 1.0;
                public static final double WHEEL_DIAMETER_INCHES = 4.0;
                public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_NEVEREST * DRIVE_GEAR_REDUCTION) /
                        (WHEEL_DIAMETER_INCHES * 3.1415);                   // Conversion: Encoder Count to Inches
                public static final double COUNTS_PER_TETRIX_INCH = (COUNTS_PER_MOTOR_TETRIX * DRIVE_GEAR_REDUCTION) /
                        (WHEEL_DIAMETER_INCHES * 3.1415);               // Conversion: Encoder Counts Motor Tetrix to Inches

                public static final int BLUE_COLOR_VALUE = 1;
                public static final int RED_COLOR_VALUE= 1;

                public static final boolean JEWEL_SENSOR_LED_ON = true;


            //--------------------------TELE-OP VALUES-------------------------
                public static final double ROTATION_SPEED = 0.4;
                public static final double DEAD_ZONE_SIZE = 0.1;
                public static final double D_PAD_SPEED = 0.2;
        //--------------------------SERVO CONFIGURATIONS-----------------

            //--------------Jewel System------------------

                //  Minimum Positions
                public static final double MIN_POSITION_DOWN = 0;
                public static final double MIN_POSITION_FLICK = 0;

                //  Maximum Positions
                public static final double MAX_POSITION_DOWN = 1;
                public static final double MAX_POSITION_FLICK = 1;

                //  Initial Positions
                public static final double START_POSITION_DOWN = 0.63;
                public static final double START_POSITION_FLICK = 0.35;

                //  Significant Positions

                    //  Centered Positions
                    public static final double CENTER_POSITION_DOWN = 0.63;
                    public static final double CENTER_POSITION_FLICK = 0.46;

                    //  Flick Positions
                    public static final double LOW_POSITION_DOWN = 0.05;
                    public static final double RIGHT_POSITION_FLICK = CENTER_POSITION_FLICK - 0.36;
                    public static final double LEFT_POSITION_FLICK = CENTER_POSITION_FLICK + 0.36;


                 //  Increments
                    public static final double INCREMENT_POSITION_DOWN = 0.01;
                    public static final long INCREMENT_FREQUENCY_DOWN = 50;


            //--------------Relic System------------------

                //  Minimum Positions
                public static final double MIN_POSITION_CLAW = 0;
                public static final double MIN_POSITION_WRIST = 0;

                //  Maximum Positions
                public static final double MAX_POSITION_CLAW = 1;
                public static final double MAX_POSITION_WRIST = 1;

                //  Initial Positions
                public static final double START_POSITION_CLAW = 0;
                public static final double START_POSITION_WRIST = 0;

                //  Significant Positions

                    //  Grab/Pick Positions
                    public static final double GRAB_POSITION_CLAW = 0;
                    public static final double OPEN_POSITION_CLAW = 0;

                    public static final double LOW_POSITION_WRIST = 0;
                    public static final double HIGH_POSITION_WRIST = 0;

                //  Increments
                    public static final double INCREMENT_POSITION_CLAW = 0.01;
                    public static final double INCREMENT_POSITION_WRIST = 0.01;




    //--------------------------ENUMERATIONS---------------------
            public enum movements{
                forward(1, -1, 1, -1),
                backward(-1, 1, -1, 1),
                right(-1, -1, 1, 1),
                left(1, 1, -1, -1),
                tr(0, -1, 1, 0),
                tl(1, 0, 0, -1),
                br(-1, 0, 0, 1),
                bl(0, 1, -1, 0),
                cw(-1, -1, -1, -1),
                ccw(1, 1, 1, 1),
                glyphUp,
                glyphDown,
                treadUp,
                treadDown,
                relicOut,
                relicIn;

                private final double[] directions;

                movements(double... signs){
                    this.directions = signs;
                }

                private double[] getDirections(){
                    return directions;
                }
            }

            public enum treadPivotPositions{
                lift,
                drop,
                center;

            }

            public enum setupType{
                all, glyph, jewel, relic, drive, teleop;
            }
            public enum team{
                red1, red2, blue1, blue2;
            }

            public enum flick{
                right,left
            }
            public enum EncoderMode{
                ON, OFF;
            }

//------------------------CONFIGURATIONS----------------------

    //  Drivetrain
        public DcMotor motorFR;
        public DcMotor motorFL;
        public DcMotor motorBR;
        public DcMotor motorBL;

        public static final String motorFRS = "motorFR";
        public static final String motorFLS = "motorFL";
        public static final String motorBRS = "motorBR";
        public static final String motorBLS = "motorBL";

    //  Jewel Systems
        public Servo jewelDown;
        public Servo jewelFlick;
        public ColorSensor jewelSensor;

        public static final String jewelDownS = "jewelDown";
        public static final String jewelFlickS = "jewelFlick";
        public static final String jewelSensorS = "colorSensor";

    //  Glyph System
        public Servo pullServo;
        public DcMotor rightTread;
        public DcMotor leftTread;
        public TouchSensor glyphButtonPick;
        public TouchSensor glyphButtonDrop;


        public static final String pullServoS = "pullServo"; // updated //Configured
        public static final String rightTreadS = "rightTread"; // updated //Configured
        public static final String leftTreadS = "leftTread"; // updated //Configured
        public static final String glyphButtonPickS = "glyphButton";
        public static final String glyphButtonDropS = "glyphButton";


    //  Relic Systems
        public DcMotor relicMotor;
        public Servo angleServo;
        public Servo rightClaw;
        public Servo leftClaw;

        public static final String relicMotorS = "relicMotor"; // updated //Configured
        public static final String angleServoS = "angleServo";
        public static final String rightClawS = "rightClaw";
        public static final String leftClawS = "leftClaw";



    //------------------ARRAYS------------
    public movements[] allMovements = {movements.forward, movements.backward, movements.right, movements.left, movements.tr, movements.tl, movements.br, movements.bl, movements.cw, movements.ccw};
    public DcMotor[] drivetrain = new DcMotor[4];
    public DcMotor[] glyphSystem = new DcMotor[3];

    public Servo[] relicSystem = new Servo[2];
    public Servo[] jewelSystem = new Servo[2];

    //----------------------------------------------------------------------------------------------------------------------------------------------------------

    public Central(){}

    public void CentralClass(setupType setup) throws InterruptedException{
        switch (setup){
            case all:
                setupDrivetrain();
                setupJewel();
                setupGlyph();
                setupRelic();
                break;
            case teleop:
                setupDrivetrain();
                setupJewel();
                setupGlyph();
                setupRelic();
                break;
            case drive:
                setupDrivetrain();
                break;
            case jewel:
                setupJewel();
                break;
            case relic:
                setupRelic();
                break;
            case glyph:
                setupGlyph();
                break;
        }
    }

    public void runOpMode() throws InterruptedException{
        CentralClass(setupType.all);
    }

    //------------------TEST FUNCTIONS------------------------------------------------------------------------


    //------------------ENCODERS MOVEMENTS------------------------------------------------------------------------

    public void driveTrainEncoderMovement(double speed, double distance, double timeoutS, long waitAfter, Central.movements movement) throws  InterruptedException{

        int[] targets = new int[drivetrain.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH);
            }
            for (DcMotor motor: drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:drivetrain){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
                for (DcMotor motor: drivetrain){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            for (DcMotor motor: drivetrain){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            sleep(waitAfter);


        }
    }
    public void encoderMovement(double speed, double distance, double timeoutS, long waitAfter, Central.movements movement, DcMotor... motors) throws  InterruptedException{

        int[] targets = new int[motors.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            sleep(waitAfter);


        }
    }


    //------------------JEWEL FUNCTIONS------------------------------------------------------------------------
    public void centerFlicker(long waitAfter){
        jewelDown.setPosition(CENTER_POSITION_DOWN);
        jewelFlick.setPosition(CENTER_POSITION_FLICK);
        sleep(800 + waitAfter);
    }
    public void initialPositionFlicker(long waitAfter){
        jewelDown.setPosition(START_POSITION_DOWN);
        jewelFlick.setPosition(START_POSITION_FLICK);
        sleep(200);
    }
    public void sweepServo(Servo servo, double EndPosition, double increment, long incrementSpeed) throws InterruptedException{
        if (servo.getPosition() > EndPosition){
            for (double p = servo.getPosition(); servo.getPosition() > EndPosition; p-= increment){
                servo.setPosition(p);
                sleep(incrementSpeed);
            }
        }else {
            for (double p = servo.getPosition(); servo.getPosition() < EndPosition; p+= increment){
                servo.setPosition(p);
                sleep(incrementSpeed);
            }
        }

    }
    public void flick(team side) throws InterruptedException{
        centerFlicker(10);

        sweepServo(jewelDown, LOW_POSITION_DOWN, INCREMENT_POSITION_DOWN, INCREMENT_FREQUENCY_DOWN);

        jewelSensor.enableLed(JEWEL_SENSOR_LED_ON);
        sleep(1500);
        telemetry.addData("Blue Value: ", jewelSensor.blue());
        telemetry.update();
        boolean loopquit= true;
        switch (side){
            case red1:
            case red2:
                while(loopquit) {
                    if (jewelSensor.blue() >= BLUE_COLOR_VALUE) { //FLICK REG
                        flick(flick.left);
                        loopquit=false;

                    } else if (jewelSensor.red() >= RED_COLOR_VALUE) {                               //FLICK OPPOSITE
                        flick(flick.right);
                        loopquit=false;
                    }else{driveTrainEncoderMovement(0.2, 0.2, 0, 0, backward);}
                }
                break;
            case blue1:
            case blue2:
                while(loopquit) {
                    if (jewelSensor.blue() >= BLUE_COLOR_VALUE) { //FLICK REG
                        flick(flick.right);
                        loopquit = false;

                    } else if (jewelSensor.red() >= RED_COLOR_VALUE) {                               //FLICK OPPOSITE
                        flick(flick.left);
                        loopquit = false;
                    }else {driveTrainEncoderMovement(0.2, 0.2, 0, 0, backward);}
                }
        }
        sleep(1000);
        centerFlicker(0);
    } // TO BE FIXED
    public void flick(flick side) throws InterruptedException{
        switch (side){
            case left:
                jewelFlick.setPosition(LEFT_POSITION_FLICK);
                telemetry.addData("Position:", jewelFlick.getPosition());
                telemetry.update();

                break;
            case right:
                jewelFlick.setPosition(RIGHT_POSITION_FLICK);
                telemetry.addData("Position:", jewelFlick.getPosition());
                telemetry.update();
                break;
        }
    }


    //------------------SET FUNCTIONS------------------------------------------------------------------------
    public void setRuntime(ElapsedTime time) throws InterruptedException{
        runtime = time;
    }

    //------------------RELIC FUNCTIONS------------------------------------------------------------------------
    //none right now

    //------------------GLYPH FUNCTIONS------------------------------------------------------------------------
    //none right now

    //------------------SERVO FUNCTIONS------------------------------------------------------------------------
    //none right now

    //------------------HARDWARE SETUP FUNCTIONS------------------------------------------------------------------------
    public DcMotor motor(DcMotor motor, HardwareMap hardwareMap, String name, DcMotor.Direction direction) throws InterruptedException{
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        return motor;
    }
    public void motorDriveMode(EncoderMode mode, DcMotor... motor) throws InterruptedException{
        switch (mode){
            case ON:
                for (DcMotor i: motor){
                    i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                idle();
                for (DcMotor i: motor){
                    i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case OFF:
                break;
        }

        this.drivetrain = motor;

    }
    public void motorEncoderMode(DcMotor... motor) throws InterruptedException{
        for (DcMotor i: motor){
            i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        idle();
        for (DcMotor i: motor){
            i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public Servo servo(Servo servo, HardwareMap hardwareMap, String name, Servo.Direction direction, double min, double max, double start) throws InterruptedException{
        servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }
    public ColorSensor colorSensor(ColorSensor sensor, HardwareMap hardwareMap, String name, boolean ledOn) throws InterruptedException{
        sensor = hardwareMap.colorSensor.get(name);
        sensor.enableLed(ledOn);

        telemetry.addData("Beacon Red Value: ", sensor.red());
        telemetry.update();

        return sensor;
    }

    public void powerMotors(double speed, long time, DcMotor... motors){
        for (DcMotor motor: motors){
            motor.setPower(speed);
        }
        sleep(time);
        for (DcMotor motor: motors){
            motor.setPower(0);
        }
    }
    public void setupDrivetrain() throws InterruptedException{
        motorFR = motor(motorFR, hardwareMap, motorFRS, DcMotorSimple.Direction.FORWARD);
        motorFL = motor(motorFL, hardwareMap, motorFLS, DcMotorSimple.Direction.FORWARD);
        motorBR = motor(motorBR, hardwareMap, motorBRS, DcMotorSimple.Direction.FORWARD);
        motorBL = motor(motorBL, hardwareMap, motorBLS, DcMotorSimple.Direction.FORWARD);

        motorDriveMode(EncoderMode.ON, motorFR, motorFL, motorBR, motorBL);
    }
    public void setupRelic() throws InterruptedException{}// FINISH
    public void setupJewel() throws InterruptedException{
        jewelDown = servo(jewelDown, hardwareMap, jewelDownS, Servo.Direction.FORWARD, MIN_POSITION_DOWN, MAX_POSITION_DOWN, START_POSITION_DOWN);
        jewelFlick = servo(jewelFlick, hardwareMap, jewelFlickS, Servo.Direction.FORWARD, MIN_POSITION_FLICK, MAX_POSITION_FLICK, START_POSITION_FLICK);

        jewelSensor = colorSensor(jewelSensor, hardwareMap, jewelSensorS, JEWEL_SENSOR_LED_ON);

    }
    public void setupGlyph() throws InterruptedException{}

    //------------------DRIVETRAIN TELEOP FUNCTIONS------------------------------------------------------------------------
    public void driveTrainMovement(double speed, Central.movements movement) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x]* speed);

        }
    }
    public void driveTrainMovementAccelerate(double speed, Central.movements movement) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (double i = 0; i <= speed; i+=.05) {
            for (DcMotor motor : drivetrain) {
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setPower(signs[x] * i);

            }
        }
    }

    public void stopDrivetrain() throws InterruptedException{
        for (DcMotor motor: drivetrain){
            motor.setPower(0);
        }
    }


}
