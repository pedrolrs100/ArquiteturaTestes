package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.BracoGarra;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Controller.Condicoes.Condicoes;

public class CondicoesBraco extends Condicoes {
    public double correnteLimite;
    public double limiteMecanico;
    public  CondicoesBraco(
            DcMotorEx motor,
            int alvo,
            ElapsedTime time,
            double correnteLimite,
            double limiteMecanico

    ){
        super(motor, alvo, time);
        this.correnteLimite = correnteLimite;
        this.limiteMecanico = limiteMecanico;
    }
    private boolean EstateLimiteMecanico () {return motor.getCurrentPosition()>limiteMecanico;}

    private boolean EstateCorrenteLimite (){
        boolean condicaoParadaSurtoEnergia = (motor.getCurrent(CurrentUnit.AMPS) >= correnteLimite);

        if(condicaoParadaSurtoEnergia && this.time.time() > 0.1 && motor.getCurrentPosition() < 230) {
            return true;
        }
        return false;
    }
    @Override
    public boolean condicaoParadaFinal(){
        if(EstateCorrenteLimite()) return true;

        if(EstateLimiteMecanico()) return true;

        return false;
    }
}
