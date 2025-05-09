package org.firstinspires.ftc.teamcode.robot.Sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.HardwareNames;

@Config
public class SensorToque {
    TouchSensor touchSensor;
    public SensorToque(HardwareMap hardwareMap){
        touchSensor = hardwareMap.get(TouchSensor.class, HardwareNames.touchSensor);
    }
    public double isPressed(){
        if(touchSensor.isPressed()){
            return 1;
        }
        return 0;
    }
}


