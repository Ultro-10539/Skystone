package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class UltroMotor extends DcMotorImplEx {
    private final ExecutorService service = Executors.newFixedThreadPool(1);
    public UltroMotor(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
    }

    public UltroMotor(DcMotorController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }

    public UltroMotor(DcMotorController controller, int portNumber, Direction direction, MotorConfigurationType motorType) {
        super(controller, portNumber, direction, motorType);
    }


    /**
     * Accelerates instead of setting power.
     * @param newPower
     */
    @Override
    public void setPower(final double newPower) {
        final double oldPower = getPower();

        double deltaPower = newPower - oldPower;
        long deltaTimeMilles = 10;

        double rate = deltaPower/(double) deltaTimeMilles;

        service.submit(() -> {
            double curr = oldPower;
            while(curr < newPower) {
                curr += rate;

                double minimize = Range.clip(curr, -1, 1);
                super.setPower(minimize);
                try {
                    Thread.sleep(deltaTimeMilles);
                }catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
    }
}
