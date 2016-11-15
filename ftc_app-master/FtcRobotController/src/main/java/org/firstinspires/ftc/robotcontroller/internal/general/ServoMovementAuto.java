package org.firstinspires.ftc.robotcontroller.internal.general;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 7260 on 12/10/2015.
 */
public class ServoMovementAuto {
    public Servo servo;

    public ServoMovementAuto(Servo servo) {
        this.servo = servo;
    }

    public void goToPos(double nPos) {
        if(nPos < servo.getPosition()) {
            for(double n = nPos; servo.getPosition() < n; n += .005) {
                servo.setPosition(servo.getPosition() + .05);
            }
        } else {
            for(double n = nPos; n > servo.getPosition(); n -= .005) {
                servo.setPosition(servo.getPosition() - .005);
            }
        }
    }
}
