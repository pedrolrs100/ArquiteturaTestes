package org.firstinspires.ftc.teamcode.robot.Sensors;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;
//import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V2;
import org.firstinspires.ftc.teamcode.common.controls.ColorMatcher;

public class SensorCor {
    //private V2 robot;
    private ColorRangeSensor sensorColor;
    int red,green,alpha,blue;
    double distance;

    public ColorMatcher colorMatcher;
    public SensorCor(HardwareMap hardwareMap, String hardwareName){
        sensorColor = hardwareMap.get(ColorRangeSensor.class, hardwareName);
        colorMatcher = new ColorMatcher(this);
    }

    public double getDistance(){this.distance = sensorColor.getDistance(DistanceUnit.CM); return this.distance;}
    public int getGreen(){ this.green = sensorColor.green(); return this.green; }
    public int getRed(){ this.red = sensorColor.red(); return this.red;}
    public int getBlue(){ this.blue = sensorColor.blue(); return this.blue;}
    public int getAlpha(){ this.alpha = sensorColor.alpha(); return this.alpha;}


}
