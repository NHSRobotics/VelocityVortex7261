package org.firstinspires.ftc.robotcontroller.internal.VelocityVortex;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.general.Movement;
import org.firstinspires.ftc.robotcontroller.internal.general.MovementAuto;

import for_camera_opmodes.OpModeCamera;

/**
 * Created by 7260 on 11/10/2016.
 */
public class SecondCompAutonomous extends OpModeCamera {

    int state = 0;
    double counts = 0;

    int i = 0;
    int angleTarget;
    int angleCurrent;
    double angleError;
    double drivePower;
    double DRIVING_CONSTANT = .01;

    int WHEEL_DIAMETER = 4;
    int ENCODER_CPR = 1120;
    double SPEED = .4;

    MovementAuto chassis;
    ModernRoboticsI2cGyro gyro;

    public void init(){
        chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"), 4, 1, 1120);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();
    }

    public void loop(){
        switch (state) {
            case 0:
                if (!gyro.isCalibrating()){
                    state++;
                }
                break;

            case 1:
                chassis.runUsingEncoders();

                chassis.forwardDrive(SPEED);

                counts = ENCODER_CPR * (60 / (Math.PI * WHEEL_DIAMETER));

                if (chassis.checkDistance(counts)){
                    chassis.resetEncoders();
                    state++;
                }
                break;

            case 2:
                if(chassis.checkEncoderReset()){
                    resetStartTime();
                    i = 0;
                    angleTarget = -49;
                    state++;
                }
                break;

            case 3:
                chassis.runUsingEncoders();

                angleCurrent = gyro.getIntegratedZValue();      //Gets the current angle

                angleError = Math.abs(angleTarget - angleCurrent);        //Finds the current difference between the target and current

                drivePower = angleError*DRIVING_CONSTANT;       //Calculates the speed based on the current difference

                //
                //Makes sure the drive power doesn't exceed a range that would throw an error
                //
                if (drivePower > 1) {
                    drivePower = 1;
                } else if (drivePower < 0) {
                    drivePower = 0;
                }

                //
                //Sets the chassis to spin clockwise
                //
                chassis.leftSpin(drivePower);
                //
                //Checks to see if the bot has reached or exceed its angle target
                //
                //Also, it checks if the code has gotten stuck in a loop. Proceeds if it has taken longer than 2 seconds
                //
                //Count is incremented to 5 to allow for small delay
                //
                //Stops the motors and resets the encoders
                //
                if ((gyro.getIntegratedZValue() <= angleTarget) && (getRuntime() > 2)){
                    if ( i == 5) {
                        chassis.stopDrive();
                        chassis.resetEncoders();
                        state++;
                    } else{
                        i++;
                    }
                }
                break;

            case 4:

                break;

            default:
                break;
        }

        telemetry.addData("State: ", state);
        telemetry.addData("Target: ", counts);
        telemetry.addData("RightCurrent: ", chassis.rightMotor.getCurrentPosition());
        telemetry.addData("LeftCurrent: ", chassis.leftMotor.getCurrentPosition());
        telemetry.addData("Z Value: ", gyro.getIntegratedZValue());
        telemetry.addData("Is gyro calibrating: ", gyro.isCalibrating());

    }
}
