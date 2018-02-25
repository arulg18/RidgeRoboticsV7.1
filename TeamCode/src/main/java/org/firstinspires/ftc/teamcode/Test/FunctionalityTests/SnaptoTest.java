package org.firstinspires.ftc.teamcode.Test.FunctionalityTests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.TeleOp.DriveMode;


/**
 * Patty Feng night b4 states
 */
@Autonomous(name = "Snap to Axis Test", group = "Test")
public class SnaptoTest extends DriveMode{

    public ElapsedTime runtime = new ElapsedTime();

    public void pseudotester() throws InterruptedException{
        snapto();
    }
}