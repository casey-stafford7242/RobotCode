package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ExternalTeamCode.ServoChecker;


public class DiagonalAutonomousRed extends OpMode
{
    DcMotor backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor, whiskMotor, leftShootMotor, rightShootMotor;
    Servo leftButtonPushServo;
    ColorSensor leftButtonPushColorSensor;
    GyroSensor gyro;
    OpticalDistanceSensor colorSensor;
    boolean timeBeenSet = false;
    boolean buttonPressed = false;
    double speedCheckStartTime;
    int currentCounts;
    private static final int MAX_MOTOR_RPM = 77;
    boolean speedCheckTrigger = false;
    double startMethodTime = 0;
    double leftServoOutPosition = .7;
    double leftServoInPosition = 1;
    BotState curState;
    ServoChecker servoChecker;


    public enum BotState
    {
        SHOOT_PARTICLES, INITIAL_TURN, DRIVE_TO_WALL, TURNS_WITH_WALL, FIND_WHITE_LINE, PUSH_BUTTON
    }

    @Override
    public void init()
    {
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        leftButtonPushServo = hardwareMap.servo.get("leftButtonPushServo");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        colorSensor = hardwareMap.opticalDistanceSensor.get("colorSensor");
        leftShootMotor = hardwareMap.dcMotor.get("leftShootMotor");
        rightShootMotor = hardwareMap.dcMotor.get("rightShootMotor");
        whiskMotor = hardwareMap.dcMotor.get("whiskMotor");
        leftButtonPushColorSensor = hardwareMap.colorSensor.get("leftButtonPushColorSensor");
        //rightButtonPushServo.setPosition(0);
        leftButtonPushServo.setPosition(1);
        // leftTouchSensor = hardwareMap.get(ModernRoboticsDigitalTouchSensor.class, "leftTouchSensor");
        // rightTouchSensor = hardwareMap.get(ModernRoboticsDigitalTouchSensor.class, "rightTouchSensor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        curState = BotState.SHOOT_PARTICLES;
    }

    @Override
    public void loop()
    {
        switch (curState)
        {
            case SHOOT_PARTICLES:
                shootParticles();
                break;
            case INITIAL_TURN:
                //Turn 45 Degrees Counter-Clockwise
                gyroTurn(-.3, 315, 5);
                break;
            case DRIVE_TO_WALL:
                driveUsingTime(2, .5);
                break;
            case TURNS_WITH_WALL:
                //Turn to realign with the wall
                gyroTurn(.3, 0, 5);
                break;
            case FIND_WHITE_LINE:
                runToWhiteLine(.05, .3);
                break;
            case PUSH_BUTTON:
                pushButton();
                break;
        }
    }

    public void sleep(long time)
    {
        //Faster than writing the exception handling each time
        try
        {
            Thread.sleep(time);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
    }

    public void gyroTurn(double motorPower, int gyroTarget, int TOLERANCE)
    {
        if(gyro.getHeading() < gyroTarget + TOLERANCE && gyro.getHeading() > gyroTarget - TOLERANCE)
        {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            if(curState == BotState.INITIAL_TURN)
            {
                curState = BotState.DRIVE_TO_WALL;
            }
            else
            {
                curState = BotState.FIND_WHITE_LINE;
            }
            sleep(500);
        }
        else if (gyro.getHeading() > gyroTarget + TOLERANCE)
        {
            backLeftMotor.setPower(-motorPower);
            backRightMotor.setPower(motorPower);
            frontLeftMotor.setPower(-motorPower);
            frontRightMotor.setPower(motorPower);
        }
        else if (gyro.getHeading() < gyroTarget - TOLERANCE)
        {
            backLeftMotor.setPower(motorPower);
            backRightMotor.setPower(-motorPower);
            frontLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(-motorPower);
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
            startMethodTime = 0;
            timeBeenSet = false;
            if(curState == BotState.DRIVE_TO_WALL)
            {
                curState = BotState.TURNS_WITH_WALL;
            }
            else
            {
                //TODO::: ADD DRIVE_TO_CAP_BALL HERE ONCE WE CREATE IT
            }
            sleep(500);
        }
    }

    public void pushButton()
    {
        telemetry.addData("Red", leftButtonPushColorSensor.red());
        telemetry.addData("Blue", leftButtonPushColorSensor.blue());
        if(leftButtonPushColorSensor.red() > leftButtonPushColorSensor.blue())
        {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            leftButtonPushServo.setPosition(leftServoOutPosition);
            if(servoChecker.continuousServoPositionChecker(leftButtonPushServo) < .05)
            {
                curState = BotState.FIND_WHITE_LINE;
            }
        }
        else
        {
            driveUsingTime(2, .15);
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
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(0);
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
            if(gyro.getHeading() > 5)
            {
                frontLeftMotor.setPower(-.2);
                frontRightMotor.setPower(.2);
                backLeftMotor.setPower(-.2);
                backRightMotor.setPower(.2);
            }
            curState = BotState.FIND_WHITE_LINE;
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
            buttonPressed = false;
            curState = BotState.PUSH_BUTTON;
            sleep(500);
        }
        else
        {
            backRightMotor.setPower(motorPower);
            backLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(motorPower);
            frontLeftMotor.setPower(motorPower);
            if(leftButtonPushServo.getPosition() <= leftServoOutPosition)
            {
                leftButtonPushServo.setPosition(leftServoInPosition);
            }
        }
    }

    public void shootParticles()
    {
        if(time <= 3)
        {
            leftShootMotor.setPower(.25);
            rightShootMotor.setPower(.25);
            alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if(time <= 3.5 && time > 3)
        {
            whiskMotor.setPower(1);
            leftShootMotor.setPower(.25);
            rightShootMotor.setPower(.25);
            alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if(time > 3.5 && time <= 5)
        {
            whiskMotor.setPower(0);
            leftShootMotor.setPower(.25);
            rightShootMotor.setPower(.25);
            alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if(time > 5)
        {
            whiskMotor.setPower(0);
            leftShootMotor.setPower(0);
            rightShootMotor.setPower(0);
            curState = BotState.INITIAL_TURN;
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

    public double motorEncoderToPowerConverter(int numToConvert)
    {
        double convertedNum = (double)(numToConvert / MAX_MOTOR_RPM);
        return convertedNum;
    }

    public void alignMotorSpeed(DcMotor motorOne, DcMotor motorTwo)
    {
        if(motorSpeed(motorOne) > motorSpeed(motorTwo))
        {
            telemetry.addData("Aligning Motor Speed", "MotorOne > MotorTwo");
            motorOne.setPower(motorOne.getPower() - ((motorEncoderToPowerConverter(motorSpeed(motorOne)) - motorEncoderToPowerConverter(motorSpeed(motorTwo))) / 2));
            motorTwo.setPower(motorTwo.getPower() + ((motorEncoderToPowerConverter(motorSpeed(motorOne)) - motorEncoderToPowerConverter(motorSpeed(motorTwo))) / 2));
        }
        else if (motorSpeed(motorOne) < motorSpeed(motorTwo))
        {
            telemetry.addData("Aligning Motor Speed", "MotorTwo > MotorOne");
            motorTwo.setPower(motorTwo.getPower() - ((motorEncoderToPowerConverter(motorSpeed(motorTwo)) - motorEncoderToPowerConverter(motorSpeed(motorOne))) / 2));
            motorOne.setPower(motorOne.getPower() + ((motorEncoderToPowerConverter(motorSpeed(motorTwo)) + motorEncoderToPowerConverter(motorSpeed(motorOne))) / 2));
        }
        else
        {
            telemetry.addData("Motor Speeds are Aligned", "MotorOne === MotorTwo");
        }
    }
}
