package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcontroller.internal.general.Movement;

import java.util.Date;

/**
 * Created by 7260 on 1/18/2016.
 */
public class testGyro2 extends OpMode {
    Movement Chassis;
    //GyroSensor gyro;

    Date time;
    long previousTime;
    long currentTime;
    long deltaTime;

    double caliAngle;
    double currentAngle = 0;
    double deltaAngle;

    public void init(){
        Chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
        //gyro = hardwareMap.gyroSensor.get("gyro");

        time = new Date();
        previousTime = time.getTime();
        //caliAngle = gyro.getRotationFraction() * 360;
    }

    public void loop(){
        if (currentAngle < 90) {
            Chassis.leftMotor.setPower(.75);
            Chassis.rightMotor.setPower(-.75);
        } else {
            Chassis.leftMotor.setPower(0);
            Chassis.rightMotor.setPower(0);
        }

        currentTime = time.getTime();
        deltaTime = currentTime - previousTime;

        //deltaAngle = (gyro.getRotation() - caliAngle) * deltaTime;

        currentAngle += deltaAngle;
        previousTime = currentTime;
    }
}
