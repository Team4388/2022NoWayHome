package frc4388.utility;

/** Add your docs here. */
public class Gains {
  public double kP;
  public double kI;
  public double kD;
  public double kF;
  public int kIzone;
  public double kPeakOutput;
  public double kmaxOutput;
  public double kminOutput;

  /**
   * Creates Gains object for PIDs
   * @param kP The P value.
   * @param kI The I value.
   * @param kD The D value.
   * @param kF The F value.
   * @param kIzone The zone of the I value.
   * @param kPeakOutput The peak output setting the motors to run the gains at, in both forward and reverse directions. By default 1.0.
   */
  public Gains(double kP, double kI, double kD, double kF, int kIzone, double kPeakOutput)
  {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.kIzone = kIzone;
    this.kPeakOutput = kPeakOutput;
    this.kmaxOutput = this.kPeakOutput;
    this.kminOutput = -this.kPeakOutput;
  }

  /**
   * Creates Gains object for PIDs
   * @param kP The P value.
   * @param kI The I value.
   * @param kD The D value.
   * @param kF The F value.
   * @param kIzone The zone of the I value.
   * @param kMinOutput The lowest output setting to run the gains at, usually in the reverse direction. By default -1.0.
   * @param kMaxOutput The highest output setting to run the gains at, usually in the forward direction. By default 1.0.
   */
  public Gains(double kP, double kI, double kD, double kF, int kIzone, double kMinOutput, double kMaxOutput)
  {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.kIzone = kIzone;
    this.kminOutput = kMinOutput;
    this.kmaxOutput = kMaxOutput;
    this.kPeakOutput = (Math.abs(this.kminOutput) > Math.abs(this.kmaxOutput)) ? Math.abs(this.kminOutput) : Math.abs(this.kmaxOutput);
  }
}
