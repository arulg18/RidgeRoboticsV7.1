package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Blue 2", group = "Autonomous")

public class Autonomous_Blue2 extends Central {

    team side = team.blue2;
    public static final String TAG = "Vuforia VuMark Sample";
    public ElapsedTime runtime = new ElapsedTime();
    public static int position = 0;


    VuforiaLocalizer vuforia;
    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.all);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVEnEtj/////AAAAGRuqDKUaCUOqnWNCia71FEYNQd/MLe2TCFrCq7fn6emLKOR3Q0aAXrfxI1ZyrgruipioI2KnVScenAwJ7oxut6uedThCIU8pIJfTmcl7q1P/6vYlqPm/avdhwH8Qq3CuSHkx/7aqN0SKFHKhV6c1UkShZaynBNfXd9K2MpcHJCnqzBKT2La6urdGGiXj7P/bwK3K/zBmVJY3z+lnsNBz31OS5L8ihr383ZOfC2EGVglkft4ulEEPv+CsR2Oa/5EFMXYr6tNsD2aXmygWXleSUGutBjtgZT0Ebw9/IkF99TFgo0jmM0sDpav+X5/11t+Bf6ufSP9MmGssfSDr6wi2QRctDiF50aJOL/ov6boj7d8/";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");


        waitForStart();
        runtime.reset();

        relicTrackables.activate();

        while (opModeIsActive()) {

            flick(side);
            sweepServo(pullServo, LOW_POSITION_PULL, 0.04, 30);

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);

                if (RelicRecoveryVuMark.LEFT == vuMark) {    //Block should be placed in the left column
                    position = 0;
                } else if (RelicRecoveryVuMark.CENTER == vuMark) {  //Block should be placed in the center column
                    position = 1;
                } else if (RelicRecoveryVuMark.RIGHT == vuMark) { //Block should be placed in the right columm
                    position = 2;
                } else {
                    telemetry.addData("VuMark", "not visible");
                }
            }
            telemetry.update();

            driveTrainEncoderMovement(0.7, 30, 10, 100, movements.forward);
            switch(position){
                case 0:         //Left
                    driveTrainEncoderMovement(0.7, 6, 10, 100, movements.left);

                    break;
                case 1:         //Center
                    driveTrainEncoderMovement(0.7, 10, 10, 100, movements.left);


                    break;
                case 2:         //Right
                    driveTrainEncoderMovement(0.7, 14, 10, 100, movements.left);

                    break;

            }
            driveTrainEncoderMovement(0.7, 6, 10, 100, movements.forward);

            GlyphDownONALL();
            driveTrainEncoderMovement(0.7, 4, 10, 100, movements.backward);

            GlyphOFF();


            break;
        }
    }

}
