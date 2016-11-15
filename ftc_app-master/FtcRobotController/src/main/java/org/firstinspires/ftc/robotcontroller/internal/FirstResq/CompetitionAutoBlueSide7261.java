package org.firstinspires.ftc.robotcontroller.internal.FirstResq;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcontroller.internal.general.MovementAuto;

/**
 * Created by 7260 on 11/20/2015.
 */

/**
 * Autonomous program that runs to bucket on the blue-side, drops the climbers into the bucket, and lights the beacon
 */
public class CompetitionAutoBlueSide7261 extends OpMode{

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


    int state = -1;                          //Integer variable that holds the current state for the state machine

    boolean gyro1st = true;                 //Holds whether the first gyro calibration has occured

    double counts;                          //Double variable that holds the number of counts the encoders have to meet
    int ENCODER_CPR = 1120;                 //The number of counts the encoder counts for one full rotation of the motor
    int wheelDiameter = 4;                  //Wheel diameter used to calculate distance based on encoder count
    int gearRatio = 1;                      //Gear ratio used to calculate distance based on encoder count

    int angleCurrent;                       //Holds the current angle that the robot is facing, found from the gyro
    int angleTarget;                        //Holds the angle that the robot wants to face
    double angleError;                      //The difference between the current angle and target angle
    double drivePower;                      //The power at which the motors spin based upon the angle error - More error = faster spin
    double DRIVING_CONSTANT = .01;          //Constant that scales the angle error to a value that can be set to the drive power
    int i = 0;                              //Count variable for angle algorithm

    double SPEED = .7;                      //The first speed for crossing the field
    double SPEED_2 = .3;                    //The second speed for precision movement to beacon

    int colorCount = 0;
    Boolean right = false;                  //Holds whether to raise the right beacon arm
    double red = 0;                         //The R value from the RGB reading from the color sensor
    double blue = 0;                        //The B value from the RGB reading from the color sensor
    int redCount = 0;                       //A count of how many times the R value was greater than the B value
    int blueCount = 0;                      //A count of how many times the B value was greater than the R value

    @Override
    public void init(){
        //
        //Initializes and sets the hardware found through the config file
        //
        Chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"), 4, 1, 1120);
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
        dim = hardwareMap.deviceInterfaceModule.get("color dim");


        dim.setDigitalChannelMode(0, DigitalChannelController.Mode.OUTPUT);
        dim.setDigitalChannelState(0, false);

        //
        //Begins calibration of the gyro, which sets the current direction the robot is facing
        //to 0 degrees turned. The robot should be completely still during this time.
        //
        //Resets the encoder count to zero
        //
        //gyro.calibrate();
        Chassis.resetEncoders();

        //
        //Initializes the legacy module and power the ultrasonic sensor in port 5
        //
        //Initializes the ultrasonic sensor
        //
        LegacyModule legacy = hardwareMap.legacyModule.get("legacy");
        legacy.enable9v(5, true);
        eyes = hardwareMap.ultrasonicSensor.get("eyes");

        //
        //Sets the servos to their initialized positions
        //
        climberArm.setPosition(.95);
        leftsideArm.setPosition(0);
        rightsideArm.setPosition(1);
        colorArm.setPosition(1);
        rightsideButton.setPosition(1);
        leftsideButton.setPosition(0);
        mantisLeft.setPosition(1);  //Sets the left mantis arm to the backward position
        mantisRight.setPosition(0);
    }

    @Override
    public void loop(){

        //
        //State machine that executes the autonomous commands sequentially dependent on predetermined conditions
        //
        switch (state){

            //
            //Checks  if the gyro has calibrated once
            //
            //Starts to calibrate gyro a second time
            //
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
            //Runs forward 25 inches
            //
            case 1:
                Chassis.runUsingEncoders();

                //
                //Moves the chassis forward at .7 SPEED
                //
                Chassis.leftMotor.setPower(SPEED);
                Chassis.rightMotor.setPower(SPEED);

                //
                //Calculates the number of counts the encoders have to reach in order to go 25 inches
                //
                counts = ENCODER_CPR * (25 / (Math.PI * wheelDiameter)) * gearRatio;

                //
                //Checks if the motors have ran to 25 inches
                //
                //After reaching 25 inches, stops the motors and resets the encoders
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
            //Sets the target angle to 45 degrees to the left (Manually changed value to 44 to compensate in gyro discrepancy)
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
            //Runs forward 84 inches
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
            //Runs backwards 17 inches
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
            //Sets the target angle to 90 degrees clockwise (Manually changed value to 86 to compensate in gyro discrepancy)
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
            //Runs forward 24 inches
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
            //Gets the distance the bot is from the wall measured by the sonar
            //
            //Based upon the distance, the state machine transitions into the appropriate case
            //
            case 13:

                double sonarDistance = eyes.getUltrasonicLevel();   //Holds the distance of the sonar


                if (sonarDistance > 13) {   //If the distance to the wall is greater than 13 cm then it calculates the encoder counts needed to get to 11

                    counts = ENCODER_CPR * (((sonarDistance - 13) / 2.54) / (Math.PI * wheelDiameter)) * gearRatio;

                    state++;
                } else if (sonarDistance == 0) {    //If the sonar value is zero (reading error by sonar) breaks and restarts case
                    break;
                } else if (sonarDistance < 11) {    //If the distance to the wall is less than 11 cm then it calculates the encoder counts needed to get to 11

                    counts = ENCODER_CPR * (((11 - sonarDistance) / 2.54) / (Math.PI * wheelDiameter)) * gearRatio;

                    state = state + 2;
                } else {  //If the distance is between 11 and 13, the encoders are reset
                    Chassis.resetEncoders();
                    state = state + 3;
                }


                break;

            //
            //Runs forward the distance calculated from the sonar distance
            //
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

            //
            //Reverses the distance calculated from the sonar distance
            //
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

            //
            //Checks if the encoders have reset
            //
            case 16:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;




            //
            //Rotates the climber arm to dump the climbers
            //
            case 17:

                //
                //Waits 1.5 seconds for the climbers to be dumped
                //
                climberArm.setPosition(0);

                if (getRuntime() > 1.5) {
                    state++;
                    Chassis.resetEncoders();
                    resetStartTime();
                }

                break;

            //
            //Determines the color of the light on the right side of the beacon
            //
            case 18:

                colorArm.setPosition(.6);       //Extends the color sensor arm
                climberArm.setPosition(.95);    //Retracts the climber arm

                //
                //Waits 1 second for the color sensor arm to extend
                //
                //Between 1 and 2 seconds, the color sensor blue and red values are compared.
                //If the reading had more blue an increment is added to blueCount; the same is done for red
                //
                //After 2 seconds the blueCount and redCount are compared to determine if the right beacon arm should be raised
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

            //
            //Reverses 10 inches
            //
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

            //
            //Raises the right or left beacon arm determined by the color sensor algorithm
            //
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

            //
            //Runs forward 10 inches
            //
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
                    colorArm.setPosition(1);

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

            //
            //Reverses 1.5 inches
            //
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
                    colorArm.setPosition(.6);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            //
            //Sets the target angle to 180 (Manually changed to 172 to compensate in gyro discrepancy)
            //
            case 26:
                if (Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0) {
                    resetStartTime();
                    angleTarget = 172;
                    i = 0;
                    state++;
                }
                break;

            //
            //Rotates the bot to the 180 degree mark
            //
            case 27:
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

            case 28:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;


            //
            //Runs forward 20 inches
            //
            case 29:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(SPEED_2);
                Chassis.rightMotor.setPower(SPEED_2);

                counts = ENCODER_CPR * (20 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);
                    rightsideButton.setPosition(1);
                    leftsideButton.setPosition(0);
                    colorArm.setPosition(1);

                    Chassis.resetEncoders();

                    state++;
                }
                break;


            /*case 30:
                if (Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0) {
                    resetStartTime();
                    angleTarget = 258;
                    i = 0;
                    state++;
                }
                break;

            //
            //Rotates the bot 90 degrees counterclockwise
            //
            case 31:
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


            case 32:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;


            case 33:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(-SPEED_2);
                Chassis.rightMotor.setPower(-SPEED_2);

                counts = ENCODER_CPR * (2 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);
                    rightsideButton.setPosition(1);
                    leftsideButton.setPosition(0);
                    colorArm.setPosition(1);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

*/
            default:
                break;


        }

        //
        //Shows telemetry data on phone
        //
        telemetry.addData("Distance: ", eyes.getUltrasonicLevel());
        if(gyro.isCalibrating()) {
            telemetry.addData("Current Angle:", gyro.getIntegratedZValue());
        }

        colorCount = redCount + blueCount;
        telemetry.addData("Color Counts:", colorCount);
        telemetry.addData("Blue:", blueCount);
        telemetry.addData("Red:", redCount);

        telemetry.addData("RawX:", gyro.rawX());
        telemetry.addData("RawY:", gyro.rawY());
        telemetry.addData("RawZ:", gyro.rawZ());
        telemetry.addData("isCalibrating:", gyro.isCalibrating());
        telemetry.addData("Heading:", gyro.getHeading());

        telemetry.addData("State", state);
    }

}
