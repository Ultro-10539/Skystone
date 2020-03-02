package com.ultro.teamcode;

import org.firstinspires.ftc.teamcode.drive.MecanumDriver;
import org.junit.Test;

public class FieldTest {

    @Test
    public void testYea() {
        MecanumDriver driver = new MecanumDriver(true);
        driver.moveFieldCentric(-1, 0, 0, 0);
    }

}
