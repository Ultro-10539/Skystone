package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;

@Autonomous(name = "AutoRedFoundation")
public class AutonomousRedFoundation extends AutoPart1{
    @Override
    public void beforeLoop(){

    }

    @Override
    public void run(){
        DeviceMap map = DeviceMap.getInstance();
        driver.turn(1.0, -90);
    }

}
