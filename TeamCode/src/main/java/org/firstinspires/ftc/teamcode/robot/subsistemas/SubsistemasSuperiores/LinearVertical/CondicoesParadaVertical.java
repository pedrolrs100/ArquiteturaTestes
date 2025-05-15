package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Controller.Condicoes.Condicoes;

public class CondicoesParadaVertical extends Condicoes {

    public ElapsedTime time;
    public double correnteLimite;
    public int alturaMaxima;

    public CondicoesParadaVertical(
            DcMotorEx motor,
            int alvo,
            ElapsedTime time,
            double correnteLimite,
            int alturaMaxima
    ){
        super(motor,alvo,time);
        this.correnteLimite = correnteLimite;
        this.alturaMaxima = alturaMaxima;
    }
    private boolean limiteCorrete(){
        boolean condicaoParadaSurtoEnergia = (motor.getCurrent(CurrentUnit.AMPS) >= correnteLimite);

        if(condicaoParadaSurtoEnergia && time.time() > 0.1 && motor.getCurrentPosition() < 230) {
            return true;
        }
        return false;
    }
    private boolean limiteAltura(){
        return motor.getCurrentPosition() > alturaMaxima;
    }

    @Override
    public boolean condicaoParadaFinal(){
        if(super.chegouAlvoPID(30)) return true;

        if(limiteAltura()) return true;

        if(limiteCorrete()) return true;


        return false;
    }
}
