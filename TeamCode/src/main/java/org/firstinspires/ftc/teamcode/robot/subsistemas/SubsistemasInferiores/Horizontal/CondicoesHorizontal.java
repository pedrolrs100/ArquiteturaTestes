package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Horizontal;

import static org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Horizontal.LinearHorizontalMotor.limiarDistance;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Controller.Condicoes.Condicoes;
import org.firstinspires.ftc.teamcode.robot.Sensors.SensorCor;

public class CondicoesHorizontal extends Condicoes {

    public SensorCor colorSensor;
    public double correnteLimite;
    public int extensaoMaxima;
    public CondicoesHorizontal(
            DcMotorEx motor,
            int alvo,
            ElapsedTime time,
            double correnteLimite,
            int extensaoMaxima
    ){
        super(motor, alvo, time);
        this.correnteLimite = correnteLimite;
        this.extensaoMaxima = extensaoMaxima;
    }
    private boolean limiteCorrete(){
        boolean condicaoParadaSurtoEnergia = (motor.getCurrent(CurrentUnit.AMPS) >= correnteLimite);

        if(condicaoParadaSurtoEnergia && this.time.time() > 0.1 && motor.getCurrentPosition() < 230) {
            return true;
        }
        return false;
    }

    private boolean limiteExtensao(){
        return motor.getCurrentPosition() > extensaoMaxima;
    }

    private boolean sensorIsDetectingHorintontal() {
        boolean condition = false;
        condition = colorSensor.getDistance() < limiarDistance;
        return  condition;
    }

    @Override
    public boolean condicaoParadaFinal(){
        if(super.chegouAlvoPID(30)) return true;

        if(limiteExtensao()) return true;

        if(limiteCorrete()) return true;

        if(sensorIsDetectingHorintontal()) return  true;


        return false;
    }
}
