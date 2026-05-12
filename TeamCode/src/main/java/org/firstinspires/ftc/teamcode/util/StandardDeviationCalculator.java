package org.firstinspires.ftc.teamcode.util;

public class StandardDeviationCalculator {

    private static double[] getCalculationComponents(double[] dataset) {

        int idx; //making an index variable for counting purposes

        //finding mean
        double mean = 0;
        for (double data : dataset) mean+=data;
        mean/=(double)dataset.length;

        double[] deviations = new double[dataset.length];

        //calculating deviations
        idx = 0;
        for (double data : dataset) {
            deviations[idx] = data - mean;
            idx++;
        }

        //squaring deviations
        idx = 0;
        for (double dev : deviations) {
            deviations[idx] = dev*dev;
            idx++;
        }

        //summing
        double sumSquaredDeviations = 0;
        for (double dev : deviations) sumSquaredDeviations+=dev;

        return new double[] {
                sumSquaredDeviations,
                (double) deviations.length
        };

    }

    /// @return Sample standard deviation
    public static double getSample(double[] dataset) {

        final double[] calculationComponents = getCalculationComponents(dataset);

        double sumSquaredDeviations = calculationComponents[0];
        double dataSize = calculationComponents[1];

        double sample = sumSquaredDeviations / (dataSize - 1);

        //convert variances to standard deviations
        sample = Math.sqrt(sample);

        return sample;
    }

    /// @return Population standard deviation
    public static double getPopulation(double[] dataset) {

        final double[] calculationComponents = getCalculationComponents(dataset);

        double sumSquaredDeviations = calculationComponents[0];
        double dataSize = calculationComponents[1];

        double population = sumSquaredDeviations / dataSize;

        //convert variances to standard deviations
        population = Math.sqrt(population);

        return population;
    }
}
