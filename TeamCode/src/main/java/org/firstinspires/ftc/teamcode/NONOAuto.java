package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="NONOAuto11: Iterative", group="Iterative Opmode")  // @TeleOp(...) is the other common choice
//@Disabled
public class NONOAuto extends OpMode

{
    DcMotor leftDrive;
    DcMotor rightDrive;
    Servo servo;
    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cColorSensor colorsensor;
    boolean goalReached;
    double TOLERANCE;
    BotState curState;

    /*DcMotor leftFlyWheel;
    DcMotor rightFlyWheel;
    DcMotor leftLift;
    DcMotor rightLift;
    boolean flag = false;
    double startTime = 0;
    GyroSensor gyroSensor;
    */

// C++ = betterThanJava;

    public enum BotState
    {
        DRIVE_FORWARD, FIRST_GYRO_TURN, SECOND_DRIVE_FORWARD, SECOND_GYRO_TURN, FIND_COLOR
    }

    @Override
    public void init()
    {
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        servo = hardwareMap.servo.get("servo");
        colorsensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        gyro.calibrate();
        curState = BotState.DRIVE_FORWARD;
        //rightFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    //YOU MUST HAVE LOOP FUNCTION.  The program will not do anything but initialize without "loop()".
    @Override
    public void loop()
    {
        telemetry.addData("Gyro Heading", gyro.getHeading());
        switch (curState)
        {
            case DRIVE_FORWARD:
                runDriveMotors(2000, .5);
                break;
            case FIRST_GYRO_TURN:
                gyroTurn(270, .3, 5);
                break;
            case SECOND_DRIVE_FORWARD:
                runDriveMotors(2000, .5);
                break;
            case SECOND_GYRO_TURN:
                gyroTurn(0, -.3, 5);
                break;
            case FIND_COLOR:
                runToBeacon(.3);
                break;
        }
    }

    public void runDriveMotors(int encoderTarget, double motorPower)
    {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setTargetPosition(encoderTarget);
        rightDrive.setTargetPosition(encoderTarget);
        while(leftDrive.isBusy() && rightDrive.isBusy())
        {
            leftDrive.setPower(motorPower);
            rightDrive.setPower(motorPower);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void gyroTurn(int gyroTarget, double motorPower, int TOLERANCE)
    {
        if (!gyro.isCalibrating())
        {
            if ((gyro.getHeading() <= gyroTarget + TOLERANCE) && (gyro.getHeading() >= gyroTarget - TOLERANCE))
            {
                goalReached = true;
            }
            if (goalReached)
            {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            if (!goalReached)
            {
                leftDrive.setPower(motorPower);
                rightDrive.setPower(-motorPower);
            }
        }
    }

    public void runToBeacon(double motorPower)
    {
        if(colorsensor.red() < 20)
        {
            leftDrive.setPower(motorPower);
            rightDrive.setPower(motorPower);
        }
        else
        {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            servo.setPosition(1);
            try
            {
                Thread.sleep(3000);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
    }

}