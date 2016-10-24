package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 7260 on 12/10/2015.
 */
public class ServoMovement {
    public Servo servo;

    public ServoMovement(Servo servo) {
        this.servo = servo;
    }

    public void posIncrease() {
        if(servo.getPosition() <= .99){
            servo.setPosition(servo.getPosition() + .005);
        }
    }

    public void posDecrease() {
        if(servo.getPosition() >= .01) {
            servo.setPosition(servo.getPosition() - .005);
        }
    }
}
