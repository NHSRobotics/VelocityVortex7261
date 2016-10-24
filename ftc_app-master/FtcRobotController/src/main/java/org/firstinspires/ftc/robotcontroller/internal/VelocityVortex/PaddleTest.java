package org.firstinspires.ftc.robotcontroller.internal.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 7260 on 10/24/2016.
 */
public class PaddleTest extends OpMode{
    Servo paddle;

    public void init(){
        paddle = hardwareMap.servo.get("paddle");
        paddle.setPosition(.25);
    }

    public void loop(){
        if (gamepad1.a){
            paddle.setPosition(.25);
        } else if (gamepad1.b){
            paddle.setPosition(.75);
        }
    }
}
