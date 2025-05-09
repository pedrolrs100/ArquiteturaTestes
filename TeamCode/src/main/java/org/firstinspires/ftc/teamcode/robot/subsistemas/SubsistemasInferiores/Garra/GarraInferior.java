package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Garra;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.subsistemas.common.Garra.Garra;
import org.firstinspires.ftc.teamcode.robot.subsistemas.common.Garra.GarraAngulationStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.common.Garra.GarraOpeningStates;

import java.util.HashMap;

public class GarraInferior extends Garra {
    public  Servo servoRotacaoDaGarra;
    public GarraRotationInferiorStates garraRotationInferiorState = GarraRotationInferiorStates.PARALELA;
    public double ServoRotacaoInferiorPosition,ServoAberturaInferiorPoition,ServoAngulacaoPosition;
    public HashMap<GarraRotationInferiorStates, Double> mapRotation = new HashMap<>();
    public GarraInferior(HardwareMap hardwareMap) {

        super(hardwareMap, HardwareNames.aberturaGarraInferiorServo, HardwareNames.angulacaoGarraInferiorServo);

        servoRotacaoDaGarra = hardwareMap.get(Servo.class, HardwareNames.rotacaoGarraInferiorServo);
        mapRotation.put(GarraRotationInferiorStates.PARALELA, 0.316);
        mapRotation.put(GarraRotationInferiorStates.PERPENDICULAR, 0.647);


        mapOpening.put(GarraOpeningStates.OPEN, 0.0);
        mapOpening.put(GarraOpeningStates.CLOSED, 1.0);



        mapAngulation.put(GarraAngulationStates.INTAKE_SPECIMEN, 0.517);
        mapAngulation.put(GarraAngulationStates.TRANSFER,0.529);

        mapAngulation.put(GarraAngulationStates.READYTO_INTAKE, 0.676);
    }

    public Action goToReadytoIntake(){//todo okey

        return new InstantAction(() -> {
            garraRotationInferiorState = GarraRotationInferiorStates.PARALELA;
            garraOpeningState = GarraOpeningStates.OPEN;
            garraAngulationState = GarraAngulationStates.READYTO_INTAKE;

            servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));

            ServoAngulacaoPosition = mapAngulation.get(garraAngulationState);
            ServoRotacaoInferiorPosition = mapRotation.get(garraRotationInferiorState);
        });
    }
    public Action goToReadyToIntake(){//todo okey

        return new InstantAction(() -> {
            garraRotationInferiorState = GarraRotationInferiorStates.PARALELA;
            garraOpeningState = GarraOpeningStates.OPEN;
            garraAngulationState = GarraAngulationStates.READYTO_INTAKE;

            servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));

            ServoAngulacaoPosition = mapAngulation.get(garraAngulationState);
            ServoRotacaoInferiorPosition = mapRotation.get(garraRotationInferiorState);
        });
    }
    public Action goToAbrirGarra(){
        return  super.abrirGarra();
    }
    public Action goToFecharGarra(){
        return super.fecharGarra();
    }
    public Action rotacionarGarraParaPosicaoParalela() {//todo okey

        return new InstantAction(() -> {
                garraRotationInferiorState = GarraRotationInferiorStates.PARALELA;

                ServoRotacaoInferiorPosition= mapRotation.get(this.mapRotation);

        });
    }

    public Action rotacionarGarraParaPosicaoPerpendicular() {//todo okey

        return new InstantAction(() -> {
            garraRotationInferiorState = GarraRotationInferiorStates.PERPENDICULAR;
            ServoRotacaoInferiorPosition = mapRotation.get(this.mapRotation);
        });
    }

    public Action goToIntake(){//todo okey

        return new InstantAction(() -> {
            //garraRotationInferiorState = GarraRotationInferiorStates.PARALELA;
            garraOpeningState = GarraOpeningStates.CLOSED;
            garraAngulationState = GarraAngulationStates.INTAKE_SPECIMEN;

            //servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));
            ServoAngulacaoPosition = mapAngulation.get(garraAngulationState);
            ServoRotacaoInferiorPosition = mapRotation.get(garraRotationInferiorState);
        });
    }
    public Action goToTransfer(){//todo okey


        return new InstantAction(() -> {
            garraRotationInferiorState = GarraRotationInferiorStates.PARALELA;
            garraOpeningState = GarraOpeningStates.CLOSED;
            garraAngulationState = GarraAngulationStates.TRANSFER;

            servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));
            ServoAngulacaoPosition = mapAngulation.get(garraAngulationState);
            ServoRotacaoInferiorPosition = mapRotation.get(garraRotationInferiorState);

        });

    }
    public Action goToReadyTransfer(){//todo okey

        return new InstantAction(() -> {
            garraRotationInferiorState = GarraRotationInferiorStates.PARALELA;
            garraOpeningState = GarraOpeningStates.CLOSED;
            garraAngulationState = GarraAngulationStates.READY_TOTRANSFER;

            servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));
            ServoAngulacaoPosition = mapAngulation.get(garraAngulationState);
            ServoRotacaoInferiorPosition = mapRotation.get(garraRotationInferiorState);

        });

    }
    public Action goToOuttake2(){
        return new InstantAction(() -> {
           // garraOpeningState = GarraOpeningStates.OPEN;
            //servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));
            // angulacaoGarraSuperiorServo.setPosition(mapAngulation.get(garraAngulationState));
            servoAberturaDaGarra.getController().pwmDisable();
        });
    }


}
