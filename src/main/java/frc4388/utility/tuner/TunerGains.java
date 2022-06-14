package frc4388.utility.tuner;

import frc4388.utility.Gains;

public class TunerGains extends Gains {
    String id;

    public TunerGains(String id) {
        super(0, 0, 0, 0, 0, 0);
        this.id = id;
    }

    public TunerGains(String id, double kP, double kI, double kD, double kF, int kIzone, double kPeakOutput) {
        super(kP, kI, kD, kF, kIzone, kPeakOutput);
        this.id = id;
    }

    public TunerGains(String id, double kP, double kI, double kD, double kF, int kIzone, double kMinOutput, double kMaxOutput) {
        super(kP, kI, kD, kF, kIzone, kMinOutput, kMaxOutput);
        this.id = id;
    }

    public void setGains(double kP, double kI, double kD, double kF) {
        m_kP = kP;
        m_kI = kI;
        m_kD = kD;
        m_kF = kF;
    }

    public void setGains(double kP, double kI, double kD) {
        setGains(kP, kI, kD, 0);
    }
}
