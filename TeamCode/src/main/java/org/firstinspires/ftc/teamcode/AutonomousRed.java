package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutonomousRed : Iterative", group = "Iterative OpMode")
public class AutonomousRed extends OpMode
{
    DcMotor backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor, whiskMotor, leftShootMotor, rightShootMotor;
    Servo leftButtonPushServo, rightButtonPushServo;
    ColorSensor rightButtonPushColorSensor;
    GyroSensor gyro;
    OpticalDistanceSensor colorSensor;
    boolean alignedWithWall = false;
    boolean driveUsingTimeTrigger = false;
    boolean shotParticles = false;
    boolean foundWhiteLineTrigger = false;
    boolean timeBeenSet = false;
    boolean strafeUsingTimeTrigger = false;
    double speedCheckStartTime;
    int currentCounts;
    private static final int MAX_MOTOR_RPM = 77;
    boolean speedCheckTrigger = false;
    boolean pushButtonTrigger = false;
    double startMethodTime = 0;
    BotState curState;


    public enum BotState
    {
        SHOOT_PARTICLES, PRESS_BEACON_ONE, PRESS_BEACON_TWO, END
    }

    @Override
    public void init()
    {
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        leftButtonPushServo = hardwareMap.servo.get("leftButtonPushServo");
        rightButtonPushServo = hardwareMap.servo.get("rightButtonPushServo");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        colorSensor = hardwareMap.opticalDistanceSensor.get("colorSensor");
        leftShootMotor = hardwareMap.dcMotor.get("leftShootMotor");
        rightShootMotor = hardwareMap.dcMotor.get("rightShootMotor");
        whiskMotor = hardwareMap.dcMotor.get("whiskMotor");
        rightButtonPushColorSensor = hardwareMap.colorSensor.get("rightButtonPushColorSensor");
        rightButtonPushServo.setPosition(1);
        leftButtonPushServo.setPosition(0);
       // leftTouchSensor = hardwareMap.get(ModernRoboticsDigitalTouchSensor.class, "leftTouchSensor");
       // rightTouchSensor = hardwareMap.get(ModernRoboticsDigitalTouchSensor.class, "rightTouchSensor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        curState = BotState.SHOOT_PARTICLES;
    }

    @Override
    public void loop()
    {
        telemetry.addData("Color Sensor Red", rightButtonPushColorSensor.red());
        telemetry.addData("Color Sensor Blue", rightButtonPushColorSensor.blue());
        telemetry.addData("Color Sensor Green", rightButtonPushColorSensor.green());
        telemetry.addData("Light Val", colorSensor.getLightDetected());
        telemetry.addData("Gyro Heading", gyro.getHeading());

        if (backLeftMotor.getPower() == 0 && backRightMotor.getPower() == 0 && frontLeftMotor.getPower() == 0 && frontRightMotor.getPower() == 0)
        {
            gyro.calibrate();
        }

        //May be the ugliest code ever created
        if(!shotParticles)
        {
            shootParticles();
        }
        if(driveUsingTimeTrigger == false && shotParticles == true)
        {
            driveUsingTime(1.5, .5);
        }
        if(strafeUsingTimeTrigger == false && driveUsingTimeTrigger == true && shotParticles == true)
        {
            strafeUsingTime(3, .5);
        }

        if(alignedWithWall == false && shotParticles == true && driveUsingTimeTrigger == true && strafeUsingTimeTrigger == true)
        {
            alignWithWall();
        }
        if(foundWhiteLineTrigger == false && shotParticles == true && driveUsingTimeTrigger == true && strafeUsingTimeTrigger == true && alignedWithWall == true)
        {
            //Runs twice after reset of foundWhiteLineTrigger in pushButton() method
            runToWhiteLine(.04, .25);
        }
        if(foundWhiteLineTrigger == true && shotParticles == true && driveUsingTimeTrigger == true && strafeUsingTimeTrigger == true && pushButtonTrigger == false && alignedWithWall == true)
        {
            //Runs twice after reset of pushButtonTrigger in runToWhiteLine() method
            pushButton();
        }

        if(driveUsingTimeTrigger == true && strafeUsingTimeTrigger == true && foundWhiteLineTrigger == true && pushButtonTrigger == true && shotParticles == true && alignedWithWall == true)
        {
            telemetry.addData("We DID IT", "YAAAAAAAAAAY");
        }

    }


    public void alignWithWall()
    {
        if(gyro.getHeading() > 180)
        {
            backLeftMotor.setPower(.125);
            frontLeftMotor.setPower(.125);
            backRightMotor.setPower(-.125);
            frontRightMotor.setPower(-.125);
            //alignFourMotorSpeed(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
        }
        else if (gyro.getHeading() < 180 && gyro.getHeading() > 0)
        {
            backRightMotor.setPower(.125);
            frontRightMotor.setPower(.125);
            backLeftMotor.setPower(-.125);
            frontLeftMotor.setPower(-.125);
            //alignFourMotorSpeed(backRightMotor, frontRightMotor, backLeftMotor, frontLeftMotor);
        }

        if(gyro.getHeading() == 0)
        {
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            alignedWithWall = true;
        }
    }

    public void driveUsingTime(double timeCheck, double motorPower)
    {
        if(startMethodTime == 0 && timeBeenSet == false)
        {
            startMethodTime = time;
            timeBeenSet = true;
        }
        if(timeBeenSet == true && time < startMethodTime + timeCheck)
        {
            backLeftMotor.setPower(motorPower);
            backRightMotor.setPower(motorPower);
            frontLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(motorPower);
            //alignFourMotorSpeed(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
        }
        else if (timeBeenSet == true && time >= startMethodTime + timeCheck)
        {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            timeBeenSet = false;
            startMethodTime = 0;
            driveUsingTimeTrigger = true;
        }
    }

    public void pushButton()
    {
        if(rightButtonPushColorSensor.red() > rightButtonPushColorSensor.blue())
        {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            leftButtonPushServo.setPosition(.4);
            pushButtonTrigger = true;
            foundWhiteLineTrigger = false;
        }
        else
        {
            driveUsingTime(3, .3);
        }
    }

    public void strafeUsingTime(double timeCheck, double motorPower)
    {
        if(startMethodTime == 0 && timeBeenSet == false)
        {
            startMethodTime = time;
            timeBeenSet = true;
        }
        if(timeBeenSet == true && time < startMethodTime + timeCheck)
        {
            backLeftMotor.setPower(-motorPower);
            backRightMotor.setPower(motorPower);
            frontLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(-motorPower);
            //alignFourMotorSpeed(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);

        }
        else if (timeBeenSet == true && time >= startMethodTime + timeCheck)
        {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            timeBeenSet = false;
            startMethodTime = 0;
            strafeUsingTimeTrigger = true;
        }
    }

    public void alignFourMotorSpeed(DcMotor motorOne, DcMotor motorTwo, DcMotor motorThree, DcMotor motorFour)
    {
        if(motorSpeed(motorOne) != motorSpeed(motorTwo))
        {
            alignMotorSpeed(motorOne, motorTwo);
        }
        if(motorSpeed(motorThree) != motorSpeed(motorFour))
        {
            alignMotorSpeed(motorThree, motorFour);
        }
        if(motorSpeed(motorOne) == motorSpeed(motorTwo) && motorSpeed(motorThree) == motorSpeed(motorFour) && motorSpeed(motorTwo) != motorSpeed(motorThree))
        {
            alignMotorSpeed(motorTwo, motorThree);
        }
    }

    public void runToWhiteLine(double driveUsingTimeTriggerVal, double motorPower)
    {
        if(colorSensor.getLightDetected() > driveUsingTimeTriggerVal)
        {
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            foundWhiteLineTrigger = true;
            pushButtonTrigger = false;
        }
        else
        {
            backRightMotor.setPower(motorPower);
            backLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(motorPower);
            frontLeftMotor.setPower(motorPower);
            //alignFourMotorSpeed(backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor);
        }
    }

    public void shootParticles()
    {
        if(time <= 2)
        {
            leftShootMotor.setPower(.25);
            rightShootMotor.setPower(.25);
            alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if(time <= 4 && time > 2)
        {
            whiskMotor.setPower(1);
            leftShootMotor.setPower(.25);
            rightShootMotor.setPower(.25);
            alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if(time > 4 && time <= 6)
        {
            whiskMotor.setPower(0);
            leftShootMotor.setPower(.25);
            rightShootMotor.setPower(.25);
            alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if(time > 6)
        {
            whiskMotor.setPower(0);
            leftShootMotor.setPower(0);
            rightShootMotor.setPower(0);
            shotParticles = true;
        }
    }


    public int motorSpeed(DcMotor motor)
    {
        if(speedCheckTrigger == false)
        {
            speedCheckStartTime = time;
            currentCounts = motor.getCurrentPosition();
            speedCheckTrigger = true;
        }
        if(time <= speedCheckStartTime + 1)
        {
            currentCounts = motor.getCurrentPosition() - currentCounts;
        }
        else
        {
            return currentCounts;
        }
        return 0;
    }

    public double rpmToPowerConverter(int numToConvert)
    {
        double convertedNum = (double)(numToConvert / MAX_MOTOR_RPM);
        return convertedNum;
    }

    public void alignMotorSpeed(DcMotor motorOne, DcMotor motorTwo)
    {
        if(motorSpeed(motorOne) > motorSpeed(motorTwo))
        {
            telemetry.addData("Aligning Motor Speed", "MotorOne > MotorTwo");
            motorOne.setPower(motorOne.getPower() - ((rpmToPowerConverter(motorSpeed(motorOne)) - rpmToPowerConverter(motorSpeed(motorTwo))) / 2));
            motorTwo.setPower(motorTwo.getPower() + ((rpmToPowerConverter(motorSpeed(motorOne)) - rpmToPowerConverter(motorSpeed(motorTwo))) / 2));
        }
        else if (motorSpeed(motorOne) < motorSpeed(motorTwo))
        {
            telemetry.addData("Aligning Motor Speed", "MotorTwo > MotorOne");
            motorTwo.setPower(motorTwo.getPower() - ((rpmToPowerConverter(motorSpeed(motorTwo)) - rpmToPowerConverter(motorSpeed(motorOne))) / 2));
            motorOne.setPower(motorOne.getPower() + ((rpmToPowerConverter(motorSpeed(motorTwo)) + rpmToPowerConverter(motorSpeed(motorOne))) / 2));
        }
        else
        {
            telemetry.addData("Motor Speeds are Aligned", "MotorOne === MotorTwo");
        }
    }

}
