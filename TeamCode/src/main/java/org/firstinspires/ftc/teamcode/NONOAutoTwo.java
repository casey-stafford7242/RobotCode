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


@Autonomous(name="NONOAuto1: Iterative", group="Iterative Opmode")  // @TeleOp(...) is the other common choice
//@Disabled
public class NONOAutoTwo extends OpMode

{
    DcMotor leftDrive;
    DcMotor rightDrive;
    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cColorSensor colorsensor;

    /*DcMotor leftFlyWheel;
    DcMotor rightFlyWheel;
    DcMotor leftLift;
    DcMotor rightLift;
    boolean flag = false;
    double startTime = 0;
    GyroSensor gyroSensor;
    */

// const int C++ = betterThanJava;

    @Override
    public void init()
    {
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop()
    {
        runUsingEncoders(3000, .5, .5, leftDrive, rightDrive);
    }

    public void runUsingEncoders(int encoderTarget, double leftMotorPower, double rightMotorPower, DcMotor leftMotor, DcMotor rightMotor)
    {
        leftMotor.setTargetPosition(encoderTarget);
        rightMotor.setTargetPosition(encoderTarget);
        while(leftMotor.getTargetPosition() < encoderTarget)
        {
            leftMotor.setPower(leftMotorPower);
            rightMotor.setPower(rightMotorPower);
        }
    }
}