package team8.tuner.configv2;

import java.util.ArrayList;

public class Config extends AbstractConfig {
    public static class SparkConfig {
        public int id;
        public boolean isInverted;
    }

    public static class MasterSparkConfig extends SparkConfig {
        public double ramp, p, i, d, f, ff, a, v, armComOffset, allowableError;
        public Double armFf;
        public double voltageCompensation = 12.0;
        public double positionConversion = 1.0, velocityConversion = 1.0;
        public double minimumOutput = -1.0, maximumOutput = 1.0;
        public double startingPosition;
        public Float forwardLimit, reverseLimit;
        public boolean isBraked = true;
    }

    public MasterSparkConfig master;
    public ArrayList<SparkConfig> slaves = new ArrayList<>();

    public int xboxId;
    public boolean writeCsv = true;
    public double aSetPoint, bSetPoint, xSetPoint, ySetPoint;
}
