package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
                public static final int RED_COLOR_VALUE= 5;

                public static final boolean JEWEL_SENSOR_LED_ON = true;
                public static int count = 0;
                public static final int DEGREE_90 = 18;




    //--------------------------TELE-OP VALUES-------------------------
                public static final double ROTATION_SPEED = 1;
                public static final double DEAD_ZONE_SIZE = 0.1;
                public static final double D_PAD_SPEED = 0.4;
                public static final double CRAWL_SPEED = 0.2;

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
                public static final double START_POSITION_FLICK = 0.5-0.11;

                //  Significant Positions

                    //  Centered Positions
                    public static final double CENTER_POSITION_DOWN = 0.5;
                    public static final double CENTER_POSITION_FLICK = 0.5;

                    //  Flick Positions
                    public static final double LOW_POSITION_DOWN = 0;
                    public static final double RIGHT_POSITION_FLICK = CENTER_POSITION_FLICK - 0.36;
                    public static final double LEFT_POSITION_FLICK = CENTER_POSITION_FLICK + 0.36;


                 //  Increments
                    public static final double INCREMENT_POSITION_DOWN = 0.01;
                    public static final long INCREMENT_FREQUENCY_DOWN = 50;

            //--------------Glyph System------------------

                //  Minimum Positions
                public static final double MIN_POSITION_PULL = 0;

                //  Maximum Positions
                public static final double MAX_POSITION_PULL = 1;

                //  Initial Positions
                public static final double START_POSITION_PULL = 1;

                //  Significant Positions
                public static final double LOW_POSITION_PULL = 0.4;
                public static final double HIGH_POSITION_PULL = 1;




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
                cwback(-1,-1,0,0),
                ccwback(1,1,0,0),
                cwfront(0,0,-1,-1),
                ccwfront(0,0,1,1),

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
                all, glyph, jewel, relic, drive, teleop, notjewel;
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
            public enum turnside{
                ccw,cw;
            }
            public enum axis{
                front,center,back;
            }

//------------------------CONFIGURATIONS----------------------
    // Sensor
        public BNO055IMU imu;
        public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        Orientation current;
        float start;
        float end;
        public static boolean isnotstopped;

        public static final String imuRedS = "imu";


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
    //    public static final String rangeSensorS1 = "rangeX";
  //  public static final String rangeSensorS2 = "rangeY";
// Range Systems

  //  public SensorMRRangeSensor rangeX;
  //  public SensorMRRangeSensor rangeY;
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
        public DcMotor relicMotorOut;
        public DcMotor relicMotorIn;
        public Servo angleServo;
        public Servo Claw;

        public static final String relicMotorOutS = "relicMotorOut";
        public static final String relicMotorInS = "relicMotorIn";
        public static final String angleServoS = "angleServo";
        public static final String ClawS = "Claw";



    //------------------ARRAYS------------
    public movements[] allMovements = {movements.forward, movements.backward, movements.right, movements.left, movements.tr, movements.tl, movements.br, movements.bl, movements.cw, movements.ccw,movements.cwback,movements.ccwback,movements.ccwfront,movements.cwfront};
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
                //setupRelic();
                setupIMU();
                break;
            case notjewel:
                setupDrivetrain();
                setupGlyph();
                //setupRelic();
                setupIMU();
            case teleop:
                setupDrivetrain();
                setupJewel();
                centerFlicker(0);
                setupGlyph();
                //setupRelic();
                setupIMU();
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
    public void centerFlicker(long waitAfter) throws InterruptedException{
        jewelDown.setPosition(CENTER_POSITION_DOWN);
        jewelFlick.setPosition(CENTER_POSITION_FLICK);
        sleep(800 + waitAfter);
    }
    public void initialPositionFlicker(long waitAfter) throws InterruptedException{
        jewelDown.setPosition(START_POSITION_DOWN);
        jewelFlick.setPosition(START_POSITION_FLICK);
        sleep(200);
    }
    protected void sweepServo(Servo servo, double EndPosition, double increment, long incrementSpeed) throws InterruptedException{
        if (servo.getPosition() > EndPosition){
            for (double p = servo.getPosition(); servo.getPosition() > EndPosition; p-= increment){
                if (!opModeIsActive()){
                    break;
                }
                servo.setPosition(p);
                sleep(incrementSpeed);
            }
        }else {
            for (double p = servo.getPosition(); servo.getPosition() < EndPosition; p+= increment){
                if (!opModeIsActive()){
                    break;
                }
                servo.setPosition(p);
                sleep(incrementSpeed);
            }
        }

    }
    public void Red() throws InterruptedException{

        if (jewelSensor.red() >= RED_COLOR_VALUE) { //FLICK REG
            flick(flick.left);
            telemetry.addLine("Red");
            telemetry.update();

        } else if (jewelSensor.blue() >= BLUE_COLOR_VALUE) {                               //FLICK OPPOSITE
            flick(flick.right);
            telemetry.addLine("Blue");
            telemetry.update();
        }
        else{
            jewelFlick.setPosition(jewelFlick.getPosition() - .01);
            if (count < 7){
                count++;

                Red();
            }

        }
    }
    public void Blue() throws InterruptedException{
        if (jewelSensor.red() >= RED_COLOR_VALUE) { //FLICK REG
            flick(flick.right);
            telemetry.addLine("Red");
            telemetry.update();

        } else if (jewelSensor.blue() >= BLUE_COLOR_VALUE) {                               //FLICK OPPOSITE
            flick(flick.left);
            telemetry.addLine("Blue");
            telemetry.update();
        }
        else {
            jewelFlick.setPosition(jewelFlick.getPosition() - .01);

            if (count < 5){
                count++;
                Blue();
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
        switch (side){
            case red1:
            case red2:


                Red();
                break;
            case blue1:
            case blue2:

                Blue();
                break;
        }
        sleep(1000);
        centerFlicker(0);
    } // TO BE FIXED
    public void flick(flick side) throws InterruptedException{
        switch (side){
            case left:
                jewelFlick.setPosition(LEFT_POSITION_FLICK);
                telemetry.addData("Position:", jewelFlick.getPosition());
                telemetry.addLine("Left");
                telemetry.update();

                break;
            case right:
                jewelFlick.setPosition(RIGHT_POSITION_FLICK);
                telemetry.addData("Position:", jewelFlick.getPosition());
                telemetry.addLine("Right");

                telemetry.update();
                break;
        }
    }
    public void turn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException{
        start = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        end = start + ((direction == turnside.cw) ? target : -target);
        isnotstopped = true;
        while (isnotstopped) {
            try {
                switch (rotation_Axis) {
                    case center:
                        driveTrainMovement(speed, (direction == turnside.cw) ? movements.cw : movements.ccw);
                        break;
                    case back:
                        driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                        break;
                    case front:
                        driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                        break;
                }
            } catch (java.lang.InterruptedException e) {
                isnotstopped = false;
            }
            if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >= end){
                stopDrivetrain();

            }

        }

    }


    public void newFlick(team side) throws InterruptedException{
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
                    }
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
                    }
                }
        }
        sleep(1000);
        centerFlicker(0);

        while (!((end<=current.firstAngle+1)||end>current.firstAngle-1)&& opModeIsActive()&& isnotstopped)
        {
            current = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        try{stopDrivetrain();}
        catch(java.lang.InterruptedException e){}
    }
    public void turn(float target, turnside direction, double speed) throws InterruptedException
    {
        turn(target,direction,speed,axis.center);
    }
    public void turn(float target, turnside direction)throws InterruptedException
    {
        turn(target,direction,1);
    }
    //------------------SET FUNCTIONS------------------------------------------------------------------------
    public void setRuntime(ElapsedTime time) throws InterruptedException{
        runtime = time;
    }

    //------------------RELIC FUNCTIONS------------------------------------------------------------------------
    //none right now

    //------------------GLYPH FUNCTIONS------------------------------------------------------------------------
    public void GlyphDown() throws InterruptedException{
        pullServo.setPosition(0.4);
        sleep(500);
        powerMotors(-0.8, 2000, rightTread, leftTread);
    }
    public void GlyphDownONALL() throws InterruptedException{
        pullServo.setPosition(0.4);
        sleep(500);

        rightTread.setPower(-0.8);
        leftTread.setPower(-0.8);
    }
    public void GlyphOFF() throws InterruptedException{
        pullServo.setPosition(0.8);
        rightTread.setPower(0);
        leftTread.setPower(0);
        sleep(500);
    }

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
    /*
    public SensorMRRangeSensor rangeSensor(SensorMRRangeSensor sensor, HardwareMap hardwareMap, String name, boolean sensorOn) throws InterruptedException
    {
        sensor = hardwareMap.rangeSensor.get(name);
       // rangeSensor.engage();

       // telemetry.addData("Beacon Red Value: ", sensor.red());
      //  telemetry.update();

        return sensor;
    }*/


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
    public void setupRelic() throws InterruptedException{
        angleServo = servo(angleServo, hardwareMap, angleServoS, Servo.Direction.FORWARD, MIN_POSITION_WRIST, MAX_POSITION_WRIST, START_POSITION_WRIST);
        Claw = servo(Claw, hardwareMap, ClawS, Servo.Direction.FORWARD, MIN_POSITION_CLAW, MAX_POSITION_CLAW, START_POSITION_CLAW);
        relicMotorOut = motor(relicMotorOut, hardwareMap, relicMotorOutS, DcMotorSimple.Direction.FORWARD);
        relicMotorIn = motor(relicMotorIn, hardwareMap, relicMotorInS, DcMotorSimple.Direction.FORWARD);
    }// FINISH
    public void setupJewel() throws InterruptedException{
        jewelDown = servo(jewelDown, hardwareMap, jewelDownS, Servo.Direction.FORWARD, MIN_POSITION_DOWN, MAX_POSITION_DOWN, START_POSITION_DOWN);
        jewelFlick = servo(jewelFlick, hardwareMap, jewelFlickS, Servo.Direction.FORWARD, MIN_POSITION_FLICK, MAX_POSITION_FLICK, START_POSITION_FLICK);

        jewelSensor = colorSensor(jewelSensor, hardwareMap, jewelSensorS, JEWEL_SENSOR_LED_ON);

    }
    public void setupGlyph() throws InterruptedException{
        pullServo = servo(pullServo, hardwareMap, pullServoS, Servo.Direction.FORWARD, MIN_POSITION_PULL, MAX_POSITION_PULL, START_POSITION_PULL);
        leftTread = motor(leftTread, hardwareMap, leftTreadS, DcMotorSimple.Direction.FORWARD);
        rightTread = motor(rightTread, hardwareMap, rightTreadS, DcMotorSimple.Direction.FORWARD);
    }

    public void setupIMU() throws InterruptedException{
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true; //copypasted from BNO055IMU sample code, no clue what this does
        parameters.loggingTag          = "IMU"; //copypasted from BNO055IMU sample code, no clue what this does
        imu = hardwareMap.get(BNO055IMU.class, imuRedS);
        imu.initialize(parameters);
}

  /*  public void setupRangeSensor() throws InterruptedException{

        rangeX = rangeSensor(rangeX, hardwareMap, rangeSensorS1, JEWEL_SENSOR_LED_ON);
        rangeY = rangeSensor(rangeY, hardwareMap, rangeSensorS2, JEWEL_SENSOR_LED_ON);

    }
*/
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
        for (double i = 0; i <= speed; i+=.1) {
            for (DcMotor motor : drivetrain) {
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setPower(signs[x] * i);

            }
        }
    }

    public void stopDrivetrain() throws InterruptedException{ //why does this throw interrupted lol
        for (DcMotor motor: drivetrain){
            motor.setPower(0);
        }
    }


}
