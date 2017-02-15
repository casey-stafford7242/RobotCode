package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ExternalTeamCode.ServoChecker;


@Autonomous(name = "DiagonalAutonomousBlue", group = "AUTONOMOUSCOOODOE")
public class DiagonalAutonomousBlue extends OpMode
{
    DcMotor backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor, whiskMotor, leftShootMotor, rightShootMotor;
    DcMotorController frontDriveMotorController, backDriveMotorController;
    Servo leftButtonPushServo, rightButtonPushServo;
    ColorSensor rightButtonPushColorSensor;
    GyroSensor gyro;
    OpticalDistanceSensor colorSensor;
    boolean timeBeenSet = false;
    double speedCheckStartTime;
    double rightServoOutPosition = .5;
    double rightServoInPosition = 0;
    int currentCounts;
    private static final int MAX_MOTOR_RPM = 77;
    boolean speedCheckTrigger = false;
    boolean firstBeaconPressed = false;
    boolean secondBeaconPressed = false;
    double startMethodTime = 0;
    BotState curState;


    public enum BotState
    {
        INITIAL_DRIVE_FORWARD, SHOOT_PARTICLES, INITIAL_TURN, DRIVE_TO_WALL, TURNS_WITH_WALL,  FIND_WHITE_LINE, PUSH_BUTTON, GET_OFF_WHITE_LINE
    }

    @Override
    public void init()
    {
        frontDriveMotorController = hardwareMap.dcMotorController.get("Front Motors");
        backDriveMotorController = hardwareMap.dcMotorController.get("Back Motors");
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
        rightButtonPushServo.setPosition(rightServoInPosition);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // leftTouchSensor = hardwareMap.get(ModernRoboticsDigitalTouchSensor.class, "leftTouchSensor");
        // rightTouchSensor = hardwareMap.get(ModernRoboticsDigitalTouchSensor.class, "rightTouchSensor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        rightButtonPushColorSensor.enableLed(true);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        curState = BotState.INITIAL_DRIVE_FORWARD;
    }

    @Override
    public void loop()
    {
        telemetry.addData("Current State", curState.toString());
        telemetry.addData("Red", rightButtonPushColorSensor.red());
        telemetry.addData("Blue", rightButtonPushColorSensor.blue());
        telemetry.addData("Green", rightButtonPushColorSensor.green());
        telemetry.addData("StartMethodTimer", startMethodTime);
        telemetry.addData("Thyme", time);

        switch(curState)
        {
            case INITIAL_DRIVE_FORWARD:
                driveUsingEncoders(1300, .375);
                break;
            case SHOOT_PARTICLES:
                shootParticles();
                break;
            case INITIAL_TURN:
                gyroTurn(.2, 40, 5);
                break;
            case DRIVE_TO_WALL:
                driveUsingEncoders(3100, .375);
                break;
            case TURNS_WITH_WALL:
                gyroTurn(-.5, 0, 7);
                break;
            case FIND_WHITE_LINE:
                runToWhiteLine(.05, .175);
                break;
            case PUSH_BUTTON:
                pushButton();
                break;
            case GET_OFF_WHITE_LINE:
                driveUsingEncoders(1000, .2);
                break;
        }

    }

    public void sleep(long time)
    {
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
        setDriveMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(gyro.getHeading() < gyroTarget + TOLERANCE && gyro.getHeading() > gyroTarget - TOLERANCE)
        {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            sleep(500);
            setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(500);
            if(curState == BotState.INITIAL_TURN)
            {
                curState = BotState.DRIVE_TO_WALL;
                sleep(500);
            }
            else
            {
                curState = BotState.FIND_WHITE_LINE;
                sleep(500);
            }
        }
        else
        {
            if(curState == BotState.INITIAL_TURN)
            {
                backLeftMotor.setPower(motorPower);
                backRightMotor.setPower(-motorPower);
                frontLeftMotor.setPower(motorPower);
                frontRightMotor.setPower(-motorPower);
            }
            else
            {
                backLeftMotor.setPower(motorPower);
                backRightMotor.setPower(-motorPower);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
            }

        }
    }

    public void setDriveMotorModes(DcMotor.RunMode mode)
    {
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
    }


    public void driveUsingEncoders(int encoderTarget, double motorPower)
    {
        setDriveMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightMotor.setTargetPosition(encoderTarget);
        frontLeftMotor.setTargetPosition(encoderTarget);
        backRightMotor.setTargetPosition(encoderTarget);
        backLeftMotor.setTargetPosition(encoderTarget);

        while(backLeftMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy())
        {
            backLeftMotor.setPower(motorPower + .1);
            backRightMotor.setPower(motorPower);
            frontLeftMotor.setPower(motorPower + .1);
            frontRightMotor.setPower(motorPower);
            rightButtonPushServo.setPosition(rightServoInPosition);
        }
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        if(curState == BotState.DRIVE_TO_WALL)
        {
            curState = BotState.TURNS_WITH_WALL;
        }
        else if (curState == BotState.INITIAL_DRIVE_FORWARD)
        {
            curState = BotState.SHOOT_PARTICLES;
        }
        startMethodTime = 0;
    }


    public void pushButton()
    {
        if(rightButtonPushColorSensor.blue() > rightButtonPushColorSensor.red())
        {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rightButtonPushServo.setPosition(rightServoOutPosition);
            sleep(3500);
            if(firstBeaconPressed == false)
            {
                firstBeaconPressed = true;
            }
            curState = BotState.GET_OFF_WHITE_LINE;
        }
        else
        {
            driveUsingEncoders(250, .2);
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
            backLeftMotor.setPower(motorPower);
            backRightMotor.setPower(motorPower);
            frontLeftMotor.setPower(0);
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

    public void runToWhiteLine(double driveUsingEncodersTriggerVal, double motorPower)
    {
        setDriveMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(colorSensor.getLightDetected() > driveUsingEncodersTriggerVal && rightButtonPushServo.getPosition() < .1)
        {
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            sleep(500);
            setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(500);
            curState = BotState.PUSH_BUTTON;
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
        if(startMethodTime == 0)
        {
            startMethodTime = time;
        }

        if(time < startMethodTime + 3)
        {
            leftShootMotor.setPower(.25);
            rightShootMotor.setPower(.25);
            alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if(time <=  startMethodTime + 4.5 && time > startMethodTime + 3)
        {
            whiskMotor.setPower(-1);
            leftShootMotor.setPower(.25);
            rightShootMotor.setPower(.25);
            alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if(time > startMethodTime + 4.5 && time  <= startMethodTime + 6)
        {
            whiskMotor.setPower(0);
            leftShootMotor.setPower(.25);
            rightShootMotor.setPower(.25);
            alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if(time > startMethodTime + 6)
        {
            whiskMotor.setPower(0);
            leftShootMotor.setPower(0);
            rightShootMotor.setPower(0);
            startMethodTime = 0;
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
