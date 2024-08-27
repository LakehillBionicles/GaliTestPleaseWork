package org.firstinspires.ftc.teamcode.aiStuff;

import java.util.Random;

public class Neuron {
    Random random = new Random();
    private Double bias = random.nextDouble();
    public Double weight1 = random.nextDouble();
    private Double weight2 = random.nextDouble();
    public double compute(double input1, double input2){
        double preActivation = (this.weight1 * input1) + (this.weight2 * input2) + this.bias;
        double output = Math.signum(preActivation);
        return output;
    }
}
