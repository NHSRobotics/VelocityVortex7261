package org.firstinspires.ftc.robotcontroller.internal.FirstResq;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by 7260 on 2/25/2016.
 */
public class testAbstractedAutonomous extends OpMode {

    MovementAuto Chassis;                   //Class that holds  the driving motors
    Servo climberArm;                       //Servo that holds and drops the climbers into the bucket
    Servo colorArm;                         //Servo that extends the color sensor
    ColorSensor RGBSensor;                  //Adafruit color sensor class
    UltrasonicSensor eyes;                  //Sonar that measures the distance the bot is from the wall
    ModernRoboticsI2cGyro gyro;             //Gyro sensor that measures angle rotation, allowing for precise turning
    DeviceInterfaceModule dim;              //Device Interface module
    Servo rightsideButton;                  //Servo that controls the right arm that hits the beacon
    Servo leftsideButton;                   //Servo that controls the left arm that hits the beacon

    Servo leftsideArm;                      //Servo that rotates on the left-side of the chassis to hit the climber levers on the mountain during teleop
    Servo rightsideArm;                     //Servo that rotates on the right-side of the chassis to hit the climber levers on the mountain during teleop
    Servo mantisLeft;                       //Servo that controls the arm that raises the all-clear signal during teleop
    Servo mantisRight;                      //Servo that controls the arm that raises the all-clear signal during teleop

    public int state = 0;

    public void init(){
        Chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"), 4, 1, 1120);
        climberArm = hardwareMap.servo.get("climber");
        leftsideArm = hardwareMap.servo.get("leftFlipper");
        rightsideArm = hardwareMap.servo.get("rightFlipper");
        rightsideButton = hardwareMap.servo.get("rightButton");
        leftsideButton = hardwareMap.servo.get("leftButton");
        colorArm = hardwareMap.servo.get("colorArm");
        RGBSensor = hardwareMap.colorSensor.get("color");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();
        Chassis.resetEncoders();

        LegacyModule legacy = hardwareMap.legacyModule.get("legacy");

        legacy.enable9v(5, true);

        eyes = hardwareMap.ultrasonicSensor.get("eyes");

        climberArm.setPosition(.5);
        leftsideArm.setPosition(.58);
        rightsideArm.setPosition(0);
        colorArm.setPosition(0);
        rightsideButton.setPosition(1);
        leftsideButton.setPosition(0);
    }

    public void loop(){
        switch (state) {
            case 0:
                Chassis.resetEncoders();
                if(gyro.isCalibrating()) {
                    state++;
                }
                break;
            case 1:
                if(Chassis.forwardDriveAuto(24)){
                    state++;
                }
                break;
            case 2:
                if(Chassis.checkEncoderReset()){
                    Chassis.first = true;
                    state++;
                }
                break;
            case 3:
                if (Chassis.turnRightAuto(44, gyro)){
                    Chassis.first = true;
                    state++;
                }
                break;
            case 4:
                if(Chassis.checkEncoderReset()){
                    state++;
                }
                break;
            case 5:
                if (Chassis.forwardDriveAuto(84)){
                    state++;
                }
                break;
            case 6:
                if(Chassis.checkEncoderReset()){
                    state++;
                }
                break;
            case 7:
                if(Chassis.backwardDriveAuto(17)){
                    state++;
                }
                break;
            case 8:
                if (Chassis.checkEncoderReset()){
                    state++;
                }
                break;
            case 9:
                if (Chassis.turnRightAuto(86, gyro)){
                    Chassis.first = true;
                    state++;
                }
                break;
            case 10:
                if(Chassis.checkEncoderReset()){
                    state++;
                }
                break;
            case 11:
                if (Chassis.forwardDriveAuto(24)){
                    state++;
                }
                break;
            case 12:
                if (Chassis.checkEncoderReset()){
                    state++;
                }
            case 13:
                if(Chassis.sonarDistance(eyes) > 13){
                    if (Chassis.forwardDriveAuto(Chassis.sonarDistance(eyes) - 13)){
                        state++;
                    }
                } else if (Chassis.sonarDistance(eyes) < 11){
                    if (Chassis.backwardDriveAuto(11 - Chassis.sonarDistance(eyes))){
                        state++;
                    }
                } else {
                    state++;
                }
                break;
            case 14:
                if (Chassis.checkEncoderReset()){
                    state++;
                }
                break;
            case 15:

            default:
                break;

        }
    }
}
