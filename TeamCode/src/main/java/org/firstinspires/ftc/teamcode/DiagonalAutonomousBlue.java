package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "DiagonalAutonomous", group = "AUTONOMOUSCOOODOE")
public class DiagonalAutonomousBlue extends OpMode
{
    DcMotor backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor, whiskMotor, leftShootMotor, rightShootMotor;
    Servo leftButtonPushServo, rightButtonPushServo;
    ColorSensor rightButtonPushColorSensor;
    GyroSensor gyro;
    OpticalDistanceSensor colorSensor;
    boolean firstGyroTurnTrigger = false;
    boolean secondGyroTurnTrigger = false;
    boolean timeBeenSet = false;
    boolean strafeUsingTimeTrigger = false;
    double speedCheckStartTime;
    int currentCounts;
    private static final int MAX_MOTOR_RPM = 77;
    boolean speedCheckTrigger = false;
    double startMethodTime = 0;
    BotState curState;


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
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

        switch(curState)
        {
            case SHOOT_PARTICLES:
                shootParticles();
                break;
            case INITIAL_TURN:
                gyroTurn(.3, 45, 3);
                break;
            case DRIVE_TO_WALL:
                driveUsingTime(2, .5);
                break;
            case TURNS_WITH_WALL:
                gyroTurn(-.3, 0, 3);
                break;
            case FIND_WHITE_LINE:
                runToWhiteLine(.05, .15);
                break;
            case PUSH_BUTTON:
                pushButton();
                break;
        }

    }

    public void sleep(long time)
    {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
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
        else
        {
            backLeftMotor.setPower(motorPower);
            backRightMotor.setPower(-motorPower);
            frontLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(-motorPower);
        }
    }





    public void alignWithWall()
    {
        if(gyro.getHeading() > 180)
        {
            backLeftMotor.setPower(.2);
            frontLeftMotor.setPower(.2);
            backRightMotor.setPower(-.2);
            frontRightMotor.setPower(-.2);
            //alignFourMotorSpeed(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
        }
        else if (gyro.getHeading() < 180 && gyro.getHeading() > 0)
        {
            backRightMotor.setPower(.2);
            frontRightMotor.setPower(.2);
            backLeftMotor.setPower(-.2);
            frontLeftMotor.setPower(-.2);
            //alignFourMotorSpeed(backRightMotor, frontRightMotor, backLeftMotor, frontLeftMotor);
        }

        if(gyro.getHeading() == 0)
        {
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            if(firstGyroTurnTrigger == false)
            {
                firstGyroTurnTrigger = true;
            }
            else
            {
                secondGyroTurnTrigger = true;
            }
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
            if(curState == BotState.DRIVE_TO_WALL)
            {
                curState = BotState.TURNS_WITH_WALL;
            }
            else
            {

            }
            startMethodTime = 0;
            sleep(1000);
        }
    }

    public void pushButton()
    {
        if(rightButtonPushColorSensor.blue() > rightButtonPushColorSensor.red())
        {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            leftButtonPushServo.setPosition(.4);
            curState = BotState.FIND_WHITE_LINE;
            sleep(1000);
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
            if(gyro.getHeading() > 3)
            {
                gyroTurn(-.2, 0, 3);
            }
            else
            {
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                curState = BotState.PUSH_BUTTON;
                sleep(500);
            }

        }
        else
        {
            backRightMotor.setPower(motorPower);
            backLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(motorPower);
            frontLeftMotor.setPower(motorPower);
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
        if(time <= 4 && time > 3)
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
