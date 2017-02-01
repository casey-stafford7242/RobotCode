package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by casey stafford on 12/8/2016.
 */
@Autonomous(name="RangeSensorTest", group="Tests")
public class RangeSensorTest extends OpMode
{
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor colorSensor , bottomLeftColorSensor, bottomRightColorSensor;
    public void init()
    {
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        bottomLeftColorSensor = hardwareMap.colorSensor.get("bottomLeftColorSensor");
        bottomRightColorSensor = hardwareMap.colorSensor.get("bottomRightColorSensor");
        rangeSensor.setI2cAddress(I2cAddr.create8bit(0x42));
        bottomLeftColorSensor.setI2cAddress(I2cAddr.create8bit(0x4c));
        bottomLeftColorSensor.enableLed(true);
        bottomRightColorSensor.setI2cAddress(I2cAddr.create8bit(0x5c));
        bottomRightColorSensor.enableLed(true);
    }

    public void loop()
    {
        telemetry.addData("Range Sensor Output", rangeSensor.cmUltrasonic());
        telemetry.addData("BottomLeftColorSensorOutput", bottomLeftColorSensor.red());
        telemetry.addData("BottomRightColorSensorOutput", bottomRightColorSensor.red());
    }
}
