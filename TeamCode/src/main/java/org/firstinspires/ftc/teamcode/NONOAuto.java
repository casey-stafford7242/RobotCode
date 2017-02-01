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


@Autonomous(name="NONOAuto11: Iterative", group="Iterative Opmode")  // @TeleOp(...) is the other common choice
//@Disabled
public class NONOAuto extends OpMode

{
    DcMotor leftDrive;
    DcMotor rightDrive;
    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cColorSensor colorsensor;
    boolean goalReached;
    double TOLERANCE;

    /*DcMotor leftFlyWheel;
    DcMotor rightFlyWheel;
    DcMotor leftLift;
    DcMotor rightLift;
    boolean flag = false;
    double startTime = 0;
    GyroSensor gyroSensor;
    */

// C++ = betterThanJava;

    @Override
    public void init() {
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    //YOU MUST HAVE LOOP FUNCTION.  The program will not do anything but initialize without "loop()".
    @Override
    public void loop() {
        if (!gyro.isCalibrating()) {
            if ((gyro.getHeading() <= 90 + TOLERANCE) && (gyro.getHeading() >= 90 - TOLERANCE)) {
                goalReached = true;
            }
            if (goalReached) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            if (!goalReached) {
                leftDrive.setPower(0.175);
                rightDrive.setPower(-0.175);
            }

    /*
    public void runUsingTime(double timeTarget, double leftMotorPower, double rightMotorPower, DcMotor leftMotor, DcMotor rightMotor)
    {
        if(!flag)
        {
            startTime = time;
            flag = true;
        }
        if(time <= startTime + timeTarget)
        {
            leftMotor.setPower(leftMotorPower);
            rightMotor.setPower(rightMotorPower);
        }
        else
        {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            flag = false;
        }
    }

    public void runUsingGyro(GyroSensor gyro, int gyroTarget, double leftMotorPower, double rightMotorPower, DcMotor leftMotor, DcMotor rightMotor)
    {
        if(gyro.getHeading() >= gyroTarget);
    }
    */
        }
    }
}