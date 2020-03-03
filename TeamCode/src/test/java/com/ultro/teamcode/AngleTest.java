package com.ultro.teamcode;

import org.firstinspires.ftc.teamcode.drive.MecanumDriver;
import org.firstinspires.ftc.teamcode.drive.Vector;
import org.junit.Test;


public class AngleTest  {

    @Test
    public void testYea() {
        MecanumDriver driver = new MecanumDriver(true);
        //move(Vector.from(0, 5), 0.5, 0.7, 0);


        driver.move(Vector.from(7, 7), 1, 1, -90);
        //move(Vector.from(5, 5), 0.5, 0.7);
    }

    public void info(String msg) {
        System.out.println(msg);
    }
}
