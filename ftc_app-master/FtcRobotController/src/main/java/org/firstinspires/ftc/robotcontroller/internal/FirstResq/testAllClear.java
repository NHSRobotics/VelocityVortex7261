package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 7260 on 2/23/2016.
 */
public class testAllClear extends OpMode{
    Servo lower;
    Servo upper;

    public void init() {
        lower = hardwareMap.servo.get("lower");
        upper = hardwareMap.servo.get("upper");
    }

    public void loop() {
        if(gamepad1.left_trigger != 0) {
            if (lower.getPosition() != 1) {
                lower.setPosition(lower.getPosition() + .005);
            }
        } else if (gamepad1.right_trigger != 0){
            if(lower.getPosition() != 0){
                lower.setPosition(lower.getPosition() - .005);
            }
        } else if(gamepad1.left_bumper) {
            if (upper.getPosition() != 1) {
                upper.setPosition(upper.getPosition() + .005);
            }
        } else if (gamepad1.right_bumper){
            if(upper.getPosition() != 0){
                upper.setPosition(upper.getPosition() - .005);
            }
        }
    }
}
