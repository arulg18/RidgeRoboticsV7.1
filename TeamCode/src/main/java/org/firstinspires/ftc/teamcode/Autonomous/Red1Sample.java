package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Central;


@Autonomous(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled


public class Red1Sample{
    private ElapsedTime runtime = new ElapsedTime();
    private boolean breaker=true;
    public void runOpMode() {

            breaker = false;
            Central functionget = new Central();
            try {
                functionget.CentralClass(Central.setupType.all, Central.team.red1); //setup
            } catch (java.lang.InterruptedException e) {return;}
            try {
                functionget.flick(Central.team.red1); // flick jewel
            } catch (java.lang.InterruptedException e) {return;}
            try {
                functionget.MovetoPos(Central.CryptoboxRedX,Central.Cryptobox1Y);
            } catch (java.lang.InterruptedException e) {return;}
            try {
                functionget.absturn(-180, Central.turnside.cw, 5);
            } catch (java.lang.InterruptedException e) {return;}

    }
}
