package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "IMU Test", group = "Test")
public class Test_IMUPosition extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.all);
        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {
            //turn((float)0.75, turnside.cw);
            telemetry.addData("IMU Acceleration:s ", imu.getAcceleration());
            telemetry.addData("IMU Angular Orientation: ", imu.getAngularOrientation());
            telemetry.addData("IMU Gravity: ", imu.getGravity());
            telemetry.addData("IMU Linear Acceleration: ", imu.getLinearAcceleration());
            telemetry.addData("IMU Position: ", imu.getPosition());
            telemetry.addData("IMU Velocity: ", imu.getVelocity());
            telemetry.addData("IMU System Status: ", imu.getSystemStatus());
            telemetry.addData("IMU Calibration Status: ", imu.getCalibrationStatus());
            telemetry.update();
            sleep(3000);

        }
    }

}
