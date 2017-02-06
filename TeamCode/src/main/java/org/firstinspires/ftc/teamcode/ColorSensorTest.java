package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="Color Sensor Test", group = "TestModes")
//@Disabled
public class ColorSensorTest extends OpMode
{
    ColorSensor leftButtonPushColorSensor, rightButtonPushColorSensor;
    @Override
    public void init()
    {
        leftButtonPushColorSensor = hardwareMap.colorSensor.get("leftButtonPushColorSensor");
        rightButtonPushColorSensor = hardwareMap.colorSensor.get("rightButtonPushColorSensor");
    }

    @Override
    public void loop()
    {
        if(leftButtonPushColorSensor.red() > leftButtonPushColorSensor.blue())
        {
            telemetry.addData("Left Color Sensor Reading Red", "REEDEDEDEDED");
        }
        else if (leftButtonPushColorSensor.blue() > leftButtonPushColorSensor.red())
        {
            telemetry.addData("Left Color Sensor Reading Blue", "BLUE");
        }

        if(rightButtonPushColorSensor.red() > rightButtonPushColorSensor.blue())
        {
            telemetry.addData("Right Color Sensor Reading Red", "RED");
        }
        else if (rightButtonPushColorSensor.blue() > rightButtonPushColorSensor.red())
        {
            telemetry.addData("Right Color Sensor Reading Blue", "BLUE");
        }
    }

}
