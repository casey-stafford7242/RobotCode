package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Created by casey stafford on 1/23/2017.
 */
@Autonomous(name="Adafruit&ProportionalTest", group="Test, Edge, Christian")
public class AdafruitColorSensorTest extends OpMode
{
    AdafruitI2cColorSensor leftColorSensor, rightColorSensor;
    DcMotor leftFlyWheel, rightFlyWheel;
    double speedCheckStartTime;
    int currentCounts;
    private static final int MAX_MOTOR_RPM = 77;
    boolean speedCheckTrigger = false;
    @Override
    public void init()
    {
        leftFlyWheel = hardwareMap.dcMotor.get("leftShootMotor");
        rightFlyWheel = hardwareMap.dcMotor.get("rightShootMotor");
        leftFlyWheel.setTargetPosition(0);
        rightFlyWheel.setTargetPosition(0);
        leftColorSensor = hardwareMap.get(AdafruitI2cColorSensor.class,"leftButtonPushColorSensor");
        rightColorSensor = hardwareMap.get(AdafruitI2cColorSensor.class, "rightButtonPushColorSensor");
    }

    @Override
    public void loop()
    {
        leftFlyWheel.setPower(1);
        rightFlyWheel.setPower(1);
        telemetry.addData("Left Flywheel Motor Speed", motorSpeed(leftFlyWheel));
        telemetry.addData("Right Flywheel Motor Speed", motorSpeed(rightFlyWheel));
        alignMotorSpeed(leftFlyWheel, rightFlyWheel);
        telemetry.addData("Left Color Sensor Blue Value", leftColorSensor.blue());
        telemetry.addData("Left Color Sensor Red Value", leftColorSensor.red());
        telemetry.addData("Left Color Sensor Green Value", leftColorSensor.green());
        telemetry.addData("Right Color Sensor Blue Value", rightColorSensor.blue());
        telemetry.addData("Right Color Sensor Red Value", rightColorSensor.red());
        telemetry.addData("Right Color Sensor Green Value", rightColorSensor.green());
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
        if(motorOne.getPower() == motorTwo.getPower())
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
        else
        {
            telemetry.addData("Align Motor Power Error", "Motor Powers Not Equal");
        }
    }
}
