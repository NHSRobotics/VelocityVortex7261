package org.firstinspires.ftc.robotcontroller.internal.VelocityVortex;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.general.MovementAuto;

/**
 * Created by 7260 on 11/10/2016.
 */
public class NoCameraAutonomous extends OpMode {

    MovementAuto chassis;
    ModernRoboticsI2cGyro gyro;
    int state = 0;
    boolean first = true;

    public void init(){
        chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"), 4, 1, 1120);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

    }

    public void loop(){
        switch (state) {
            case 0:
                if(gyro.isCalibrating()){
                    state++;
                }
                break;

            case 1:
                chassis.resetEncoders();
                chassis.setDistance(12);
                chassis.forwardDriveAuto();
                state++;
                break;

            case 2:
                if(chassis.checkDistance(12)){
                    state++;
                }
                break;

            /*case 3:
                if(chassis.turnLeftAuto(45, gyro)){
                    state++;
                }*/

            default:
                break;

        }
        telemetry.addData("State: ", state);
        telemetry.addData("LeftCurrent: ", chassis.leftMotor.getCurrentPosition());
        telemetry.addData("RightCurrent: ", chassis.rightMotor.getCurrentPosition());
        telemetry.addData("LeftTarget: ", chassis.leftMotor.getTargetPosition());
        telemetry.addData("RightTarget: ", chassis.rightMotor.getTargetPosition());
        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
    }
}
