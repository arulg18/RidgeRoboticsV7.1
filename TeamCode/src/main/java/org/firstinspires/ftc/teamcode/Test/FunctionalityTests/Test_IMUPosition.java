package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "IMU Position Test", group = "Test")
public class Test_IMUPosition extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.all);
        waitForStart();
        imu.startAccelerationIntegration(new Position(),new Velocity(),500);

        runtime.reset();
        while (opModeIsActive()) {
            //turn((float)0.75, turnside.cw);
            telemetry.addData("IMU Acceleration:s ", imu.getAcceleration());
            telemetry.addData("IMU Angular Orientation: ", imu.getAngularOrientation());
            telemetry.addData("IMU Gravity: ", imu.getGravity());
            telemetry.addData("IMU Linear Acceleration: ", imu.getLinearAcceleration());
            telemetry.addData("IMU Position: ", imu.getPosition());
            telemetry.addData("IMU Position To String: ", imu.getPosition().toString());
            telemetry.addData("IMU Position To Unit: ", imu.getPosition().toUnit(DistanceUnit.INCH));
            telemetry.addData("IMU Velocity: ", imu.getVelocity());
            telemetry.addData("IMU System Status: ", imu.getSystemStatus());
            telemetry.addData("IMU Calibration Status: ", imu.getCalibrationStatus());
            telemetry.update();
            sleep(10);

        }
    }

}
