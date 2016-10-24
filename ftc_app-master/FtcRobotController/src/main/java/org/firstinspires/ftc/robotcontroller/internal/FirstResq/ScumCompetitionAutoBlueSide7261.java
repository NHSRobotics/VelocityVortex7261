package org.firstinspires.ftc.robotcontroller.internal.FirstResq;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by 7260 on 11/20/2015.
 */

/**
 * Autonomous program that runs to bucket on the blue-side, drops the climbers into the bucket, and runs up the mountain
 */
public class ScumCompetitionAutoBlueSide7261 extends OpMode{

    MovementAuto Chassis;                   //Class that holds  the driving motors
    Servo climberArm;                       //Servo that holds and drops the climbers into the bucket
    Servo leftsideArm;                      //Servo that rotates on the left-side of the chassis to hit the climber levers on the mountain
    Servo rightsideArm;
    Servo rightsideButton;
    Servo leftsideButton;
    Servo mantisLeft;
    Servo mantisRight;
    Servo colorArm;
    ColorSensor RGBSensor;
    UltrasonicSensor eyes;                  //Servo that rotates on the right-side of the chassis to hit the climber levers on the mountain
    ModernRoboticsI2cGyro gyro;             //Gyro sensor that measures angle rotation, allowing for precise turning

    int state = -1;                          //Integer variable that holds the current state for the state machine

    boolean gyro1st = true;

    double counts;                          //Double variable that holds the number of counts the encoders have to meet
    int ENCODER_CPR = 1120;                 //The number of counts the encoder counts for one full rotation of the motor
    int wheelDiameter = 4;                  //Wheel diameter used to calculate distance based on encoder count
    int gearRatio = 1;                      //Gear ratio used to calculate distance based on encoder count

    int angleCurrent;                       //Holds the current angle that the robot is facing, found from the gyro
    int angleTarget;                        //Holds the angle that the robot wants to face
    double angleError;                      //The difference between the current angle and target angle
    double drivePower;                      //The power at which the motors spin based upon the angle error - More error = faster spin
    double DRIVING_CONSTANT = .01;          //Constant that scales the angle error to a value that can be set to the drive power
    int i = 0;

    double SPEED = 1;
    double SPEED_2 = .3;

    Boolean right = false;
    double red = 0;
    double blue = 0;
    int redCount = 0;
    int blueCount = 0;

    @Override
    public void init(){
        //
        //Initializes and sets the hardware found through the config file
        //
        Chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"),4,1,1120);
        climberArm = hardwareMap.servo.get("climber");
        leftsideArm = hardwareMap.servo.get("leftFlipper");
        rightsideArm = hardwareMap.servo.get("rightFlipper");
        rightsideButton = hardwareMap.servo.get("rightButton");
        leftsideButton = hardwareMap.servo.get("leftButton");
        colorArm = hardwareMap.servo.get("colorArm");
        RGBSensor = hardwareMap.colorSensor.get("color");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        mantisLeft = hardwareMap.servo.get("mantisL");
        mantisRight = hardwareMap.servo.get("mantisR");
        //
        //Begins calibration of the gyro, which sets the current direction the robot is facing
        //to 0 degrees turned. The robot should be completely still during this time.
        //
        //Resets the encoder count to zero
        //
        //gyro.calibrate();
        Chassis.resetEncoders();

        LegacyModule legacy = hardwareMap.legacyModule.get("legacy");

        legacy.enable9v(5, true);

        eyes = hardwareMap.ultrasonicSensor.get("eyes");

        //
        //Sets the servos to their initialized positions
        //
        climberArm.setPosition(1);
        leftsideArm.setPosition(0);
        rightsideArm.setPosition(1);
        colorArm.setPosition(0);
        rightsideButton.setPosition(1);
        leftsideButton.setPosition(0);
        mantisLeft.setPosition(.3);  //Sets the left mantis arm to the backward position
        mantisRight.setPosition(.7);
    }

    @Override
    public void loop(){

        switch (state){


            case -1:
                if(gyro1st){
                    gyro.calibrate();
                    gyro1st = false;
                }
                if (gyro.isCalibrating()){
                    state++;
                }
                break;

            //
            //Waits for the gyro to finish calibrating
            //
            case 0:
                Chassis.resetEncoders();
                if(!(gyro.isCalibrating()))
                    state++;
                break;

            //
            //Runs forward 2ft
            //
            case 1:
                Chassis.runUsingEncoders();

                //
                //Moves the chassis forward at half SPEED
                //
                Chassis.leftMotor.setPower(SPEED);
                Chassis.rightMotor.setPower(SPEED);

                //
                //Calculates the number of counts the encoders have to reach in order to go 2ft
                //
                counts = ENCODER_CPR * (24 / (Math.PI * wheelDiameter)) * gearRatio;

                //
                //Checks if the motors have ran to 2ft
                //
                //After reaching 2ft, stops the motors and resets the encoders
                //
                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            //
            //Checks to see if the encoders have reset
            //
            //Resets  the current time to zero
            //
            //Resets the count to zero
            //
            //Sets the target angle to 45 degrees to the left
            //
            case 2:
                if (Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    resetStartTime();
                    i = 0;
                    angleTarget = 44;
                    state++;
                }
                break;

            //
            //Rotates the bot 45 degrees in the clockwise direction
            //
            case 3:
                Chassis.runUsingEncoders();

                angleCurrent = gyro.getIntegratedZValue();      //Gets the current angle

                angleError = angleTarget - angleCurrent;        //Finds the current difference between the target and current

                drivePower = angleError*DRIVING_CONSTANT;       //Calculates the SPEED based on the current difference

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
                Chassis.leftMotor.setPower(-drivePower);
                Chassis.rightMotor.setPower(drivePower);

                //
                //Checks to see if the bot has reached or exceed its angle target
                //
                //Also, it checks if the code has gotten stuck in a loop. Proceeds if it has taken longer than 2 seconds
                //
                //Count is incremented to 5 to allow for small delay
                //
                //Stops the motors and resets the encoders
                //
                if ((gyro.getIntegratedZValue() >= angleTarget) && (getRuntime() > 2)) {
                    if ( i == 5) {
                        Chassis.stopDrive();
                        Chassis.resetEncoders();
                        state++;
                    } else {
                        i++;
                    }
                }

                break;

            //
            //Checks to see if the encoders have reset
            //
            case 4:
                if (Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0) {
                    state++;
                }
                break;

            //
            //Runs forward 6ft
            //
            case 5:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(SPEED);
                Chassis.rightMotor.setPower(SPEED);

                counts = ENCODER_CPR * (84 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();


                    state++;
                }
                break;

            case 6:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;

            //
            //Runs backwards 1ft
            //
            case 7:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(-SPEED_2);
                Chassis.rightMotor.setPower(-SPEED_2);

                counts = ENCODER_CPR * (17 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            //
            //Checks to see if the encoders have reset
            //
            //Resets  the current time to zero
            //
            //Resets the count to zero
            //
            //Sets the target angle to 90 degrees clockwise
            //
            case 8:
                if (Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    resetStartTime();
                    angleTarget = 86;
                    i = 0;
                    state++;
                }
                break;

            //
            //Rotates the bot 90 degrees in the clockwise direction
            //
            case 9:
                Chassis.runUsingEncoders();

                angleCurrent = gyro.getIntegratedZValue();

                angleError = angleTarget - angleCurrent;

                drivePower = angleError * DRIVING_CONSTANT;

                if (drivePower > 1) {
                    drivePower = 1;
                } else if (drivePower < 0) {
                    drivePower = 0;
                }

                Chassis.leftMotor.setPower(-drivePower);
                Chassis.rightMotor.setPower(drivePower);

                if ((gyro.getIntegratedZValue() >= angleTarget) && (getRuntime() > 2)){
                    if ( i == 5) {
                        Chassis.stopDrive();
                        Chassis.resetEncoders();
                        state++;
                    } else{
                        i++;
                    }
                }

                break;

            //
            //Checks to see if the encoders have reset
            //
            case 10:
                if (Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0) {
                    state++;
                }
                break;

            //
            //Runs forward 23.75 inches
            //
            case 11:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(SPEED_2);
                Chassis.rightMotor.setPower(SPEED_2);

                counts = ENCODER_CPR * (24 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            //
            //Checks to see if the encoders have reset
            //
            //Resets  the current time to zero
            //
            case 12:
                if (Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                    resetStartTime();
                    i = 0;
                    counts = 0;
                }
                break;

            //
            //Gradually rotates the climber arm to the extended position for the climbers to be pulled off
            //
            case 13:

                double sonarDistance = eyes.getUltrasonicLevel();

                if (sonarDistance > 15) {

                    counts = ENCODER_CPR * (((sonarDistance - 15) / 2.54) / (Math.PI * wheelDiameter)) * gearRatio;

                    state++;
                } else if (sonarDistance == 0) {
                    break;
                } else if (sonarDistance < 13) {

                    counts = ENCODER_CPR * (((13 - sonarDistance) / 2.54) / (Math.PI * wheelDiameter)) * gearRatio;

                    state = state + 2;
                } else {
                    Chassis.resetEncoders();
                    state = state + 3;
                }


                break;

            case 14:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(SPEED_2);
                Chassis.rightMotor.setPower(SPEED_2);

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            case 15:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(-SPEED_2);
                Chassis.rightMotor.setPower(-SPEED_2);

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            case 16:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;




            //
            //Gradually rotates the climber arm to the extended position for the climbers to be pulled off
            //
            case 17:

                //
                //Checks if the servo has reached its position or whether the it has taken longer than 2 seconds
                //
                climberArm.setPosition(0);

                if (getRuntime() > 1.5) {
                    state++;
                    Chassis.resetEncoders();
                    resetStartTime();
                }

                break;

            case 18:

                colorArm.setPosition(1);
                climberArm.setPosition(1);

                if (getRuntime() > 1 && getRuntime() < 2) {

                    red = RGBSensor.red();
                    blue = RGBSensor.blue();

                    if (red > blue) {
                        redCount++;
                    } else {
                        blueCount++;
                    }
                }
                else if (getRuntime() > 2) {

                    if (redCount < blueCount) {
                        right = true;
                    } else {
                        right = false;
                    }

                    state++;
                }
                break;

            case 19:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;

            case 20:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(-SPEED_2);
                Chassis.rightMotor.setPower(-SPEED_2);

                counts = ENCODER_CPR * (10 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);


                    Chassis.resetEncoders();
                    resetStartTime();

                    state++;
                }
                break;

            case 21:
                if(right){
                    rightsideButton.setPosition(.52);
                } else {
                    leftsideButton.setPosition(.52);
                }


                if (getRuntime() > 1) {
                    state++;
                }
                break;

            case 22:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;

            case 23:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(SPEED_2);
                Chassis.rightMotor.setPower(SPEED_2);

                counts = ENCODER_CPR * (10 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);
                    rightsideButton.setPosition(1);
                    leftsideButton.setPosition(0);
                    colorArm.setPosition(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            //
            //Checks to see if the encoders have reset
            //
            case 24:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;

            case 25:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(-SPEED_2);
                Chassis.rightMotor.setPower(-SPEED_2);

                counts = ENCODER_CPR * (1.5 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);
                    rightsideButton.setPosition(1);
                    leftsideButton.setPosition(0);
                    colorArm.setPosition(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            case 26:
                if (Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0) {
                    resetStartTime();
                    angleTarget = -387;
                    i = 0;
                    state++;
                }
                break;

            //
            //Rotates the bot 90 degrees counterclockwise
            //
            case 27:
                Chassis.runUsingEncoders();

                angleCurrent = gyro.getIntegratedZValue();

                angleError = angleCurrent - angleTarget;

                drivePower = angleError * DRIVING_CONSTANT;

                if (drivePower > 1) {
                    drivePower = 1;
                } else if (drivePower < 0) {
                    drivePower = 0;
                }

                Chassis.leftMotor.setPower(drivePower);
                Chassis.rightMotor.setPower(-drivePower);

                if ((gyro.getIntegratedZValue() <= angleTarget) && (getRuntime() > 2)) {
                    if ( i == 5) {
                        Chassis.stopDrive();
                        Chassis.resetEncoders();
                        state++;
                    } else {
                        i++;
                    }
                }

                break;

            case 28:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;


            case 29:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(SPEED);
                Chassis.rightMotor.setPower(SPEED);

                counts = ENCODER_CPR * (60 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);
                    rightsideButton.setPosition(1);
                    leftsideButton.setPosition(0);
                    colorArm.setPosition(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;


            default:
                break;


        }

        //
        //Shows telemetry data on phone
        //
        /*telemetry.addData("Distance: ", eyes.getUltrasonicLevel());
        telemetry.addData("Current Angle:", gyro.getIntegratedZValue());
        //telemetry.addData("Difference between Target and Current:", angleError);
        //telemetry.addData("Target Angle:", angleTarget);
        //telemetry.addData("Drive Power", drivePower);

        //telemetry.addData("Encoder Count:", counts);
        //telemetry.addData("Current Encoder Count:", Chassis.leftMotor.getCurrentPosition());

        telemetry.addData("State", state);

        telemetry.addData("Climber Arm Position", climberArm.getPosition());
        telemetry.addData("Color Sensor Arm Position", colorArm.getPosition());*/
        telemetry.addData("Distance: ", eyes.getUltrasonicLevel());
        if(gyro.isCalibrating()) {
            telemetry.addData("Current Angle:", gyro.getIntegratedZValue());
        }
        telemetry.addData("RawX:", gyro.rawX());
        telemetry.addData("RawY:", gyro.rawY());
        telemetry.addData("RawZ:", gyro.rawZ());
        telemetry.addData("isCalibrating:", gyro.isCalibrating());
        telemetry.addData("Heading:", gyro.getHeading());

        telemetry.addData("State", state);
    }

}
