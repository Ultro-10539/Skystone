package com.ultro.teamcode;


import com.qualcomm.robotcore.util.Range;

import net.jafama.FastMath;

import org.firstinspires.ftc.teamcode.drive.MathUtil;
import org.firstinspires.ftc.teamcode.drive.Vector;
import org.junit.Test;
import org.junit.runners.JUnit4;

import java.util.Arrays;
import java.util.logging.Logger;

public class WrapAngleTest {

    @Test
    public void testYea() {
        Logger log = Logger.getLogger("test");


        log.info(MathUtil.wrapAngle(181) + "<-- 181");
        log.info(MathUtil.wrapAngle(-181) + "<-- -181");
    }

}
