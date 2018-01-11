package org.firstinspires.ftc.teamcode.VuforiaCodes;

import android.support.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Central;


public class GlyphVuforiaFinalRun extends Central {

    OpenGLMatrix lastLocation = null;

    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AeHK+j//////AAAAGV9JzI/9h0DLhZ7c7w4sN30lJRIpNPRyXVHdsqCX+XHpMysSwND71QWYT9YFkwVxopMQaXnzmfWK7Sc2cSJJLPU9r2G/ioxim4UU4c4rPyvhtkOcZkaS6hAPo+aKdQVUsVkBsBbPIRcQOAEmp7oKqV0d/8pydpXHCAUA18eNjdoEufCSugolPo84nHnEcEiklpqljewrCObyMTTwoftkpCEabzJoHZ5s15Ztja9s9afEXBA5Vhp2OEcdxWQVoTHL5eFJog3faeMBSyiU/NKjRNHv04w+P8lMnClXXLI8BiFVof8X5MDQPv8vFRHEADe8lCpnYDh1EGM0ZkFJv+gc59k4Ky1bIdUZYNvEto3Y5WRK\n";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);

        // Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1); //how many targets viewed at a time

        VuforiaTrackables glyphs = vuforia.loadTrackablesFromAsset("GlyphTracking_OT");

        glyphs.get(0).setName("glyphBrown");

        VuforiaTrackableDefaultListener glyphBrown = (VuforiaTrackableDefaultListener) glyphs.get(0).getListener();


        glyphs.activate();  // would be at beginning of code,
        VectorF angles = anglesFromTarget(glyphBrown);


        VectorF trans = navOffWall(glyphBrown.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));

        while (trans.get(0) > 0) {
            turn(90, turnside.ccw, 0.5);
            trans = navOffWall(glyphBrown.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));
        }

        int initialPosition = motorFR.getCurrentPosition();
        while (trans.get(2) > 3) {
            motorBL.setPower(0.2);
            motorBR.setPower(0.2);
            motorFL.setPower(0.2);
            motorFR.setPower(0.2);
            trans = navOffWall(glyphBrown.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));

        }



        double distanceTravelled = (motorFR.getCurrentPosition() - initialPosition)/ COUNTS_PER_INCH;
        while (trans.get(0) > 0) {
            trans = navOffWall(glyphBrown.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));
            turn(90, turnside.ccw, 0.5);

        }

        powerMotors(0.2, 5000, rightTread, leftTread);



    }
    @NonNull
    private VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall) {
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    @NonNull
    private VectorF anglesFromTarget(VuforiaTrackableDefaultListener image) {
        float[] data = image.getRawPose().getData();
        float[][] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};

        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float) thetaX, (float) thetaY, (float) thetaZ);
    }


}
