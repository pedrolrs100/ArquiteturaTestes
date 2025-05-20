package org.firstinspires.ftc.teamcode.robot.Controller.ControladoresDePosicao;

import com.qualcomm.robotcore.util.ElapsedTime;

@Deprecated
public class PIDTargetChecker {
    private double errorTolerance; // Margem de tolerância para o erro
    private double stabilityTime; // Tempo necessário para considerar o sistema estável
    private ElapsedTime TempoDentroDaTolerancia;

    public PIDTargetChecker(double errorTolerance, double stabilityTime) {
        this.errorTolerance = errorTolerance;
        this.stabilityTime = stabilityTime;
        this.TempoDentroDaTolerancia = new ElapsedTime();
    }

    // Pessoal esse é o javaDoc, uma forma de deixar as funções bem descritas o que elas esperam e o que retornam para que
    // outros programadores consigam usar bem a função também, muito apreciado no mercado de trabalho.
    // vamos fazer disso um padrão pra funções muito importantes e que vão er usadas por mais gente
    /**
     * Verifica se o controlador PID chegou ao alvo.
     * O método avalia se o valor atual está dentro de uma margem de tolerância em relação ao setpoint
     * e se permanece estável nessa faixa por um tempo definido.
     *
     * @param setpoint       O valor desejado que o sistema deve alcançar.
     * @param currentValue   O valor atual medido do sistema.
     * @return true se o valor atual está dentro da margem de tolerância e estabilizado pelo tempo necessário;
     *         false caso contrário.
     */
    public boolean hasReachedTarget(double setpoint, double currentValue) {

        double error = Math.abs(setpoint - currentValue);

        // Verifica se o erro está fora da tolerância
        if (error > errorTolerance) {
            // Reinicia o temporizador, pois o sistema está fora da tolerância
            TempoDentroDaTolerancia.reset();
            return false;
        }

        // Verifica se o tempo dentro da tolerância atingiu o tempo de estabilidade
        if (TempoDentroDaTolerancia.time() >= stabilityTime) {
            return true;
        }

        // Ainda dentro da tolerância, mas o tempo não foi suficiente
        return false;
    }

}
