package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by 7260 on 11/11/2015.
 */
public class Lift {
    DcMotor lift;

    public Lift(DcMotor lift) {
        this.lift = lift;
    }

    public void raise() {
        lift.setPower(.5);
    }

    public void stop() {
        lift.setPower(0);
    }

    public void lower() {
        lift.setPower(-.5);
    }
}
