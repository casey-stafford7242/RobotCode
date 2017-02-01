package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "TOOCHSUNSERRR", group = "SUNSERTESTYO840")
public class ToochSunser extends OpMode
{
    ModernRoboticsDigitalTouchSensor leftTouchSensor, rightTouchSensor;
    DcMotor backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor;
    @Override
    public void init()
    {
        leftTouchSensor = hardwareMap.get(ModernRoboticsDigitalTouchSensor.class, "leftTouchSensor");
        rightTouchSensor = hardwareMap.get(ModernRoboticsDigitalTouchSensor.class, "rightTouchSensor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        if(leftTouchSensor.isPressed() && !rightTouchSensor.isPressed())
        {
            frontRightMotor.setPower(.15);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(.15);
            backLeftMotor.setPower(0);
            telemetry.addData("Left is Pressed, Right is NO", "YEEE");
        }
        else if(!leftTouchSensor.isPressed() && rightTouchSensor.isPressed())
        {
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(.15);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(.15);
            telemetry.addData("Right is Pressed, Left is NO", "YEEE");
        }
        else if (!leftTouchSensor.isPressed() && !rightTouchSensor.isPressed())
        {
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(1);
            backLeftMotor.setPower(1);
            backRightMotor.setPower(1);
            telemetry.addData("Neither is Pressed", "YEEE");
        }
        else if (leftTouchSensor.isPressed() && rightTouchSensor.isPressed())
        {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            telemetry.addData("BOTH ARE PRESSED", "YEEE");
        }
    }
}
