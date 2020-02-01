/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


package testing;

import hallib.HalDashboard;
import hallib.HalUtil;

/*
 *   PID Control is a way to minimize error by using a formula called Proportion, Integral, Derivative.
 *   Overtime, the robot will make large errors by not being extremely accurate when going to the target value.
 *   In order to fix this, it's possible to minimize the error by adjusting the power through proportion then integral and then derivative.
 *   Each one respectively are ok in reducing error. The Proportion method drastically affects the error so it needs a lot of fine tuning.
 *   The Integral and Derivative method are fine tuning methods and are combined to create an effective way of reducing error.
 *   These three methods are used for current time processing. A fourth method called feedforward sort of acts to predict the next error and adjust for it.
 */

public class PIDControlTest {

    //private HalDashboard dashboard;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double tolerance;
    private double settlingTime;
    private PidInput pidInput;

    private boolean inverted = false;
    private boolean absSetPoint = false;
    private boolean noOscillation = false;
    private double minTarget = 0.0;
    private double maxTarget = 0.0;
    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    private double prevTime = 0.0;
    private double prevError = 0.0;
    private double totalError = 0.0;
    private double settlingStartTime = 0.0;
    private double setPoint = 0.0;
    private double setPointSign = 1.0;
    private double input = 0.0;
    private double output = 0.0;

    private double pTerm;
    private double iTerm;
    private double dTerm;
    private double fTerm;

    public interface PidInput
    {
        double getInput(PIDControlTest pidCtrl);
    }

    // Constructor
    public PIDControlTest(double kP, double kI, double kD, double kF, double tolerance, double settlingTime, PidInput pidInput) {
        //dashboard = ShadowMenuTest.getDashboard();
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.tolerance = tolerance;
        this.settlingTime = settlingTime;
        this.pidInput = pidInput;
    }

    public void displayPidInfo(int lineNum) {
        //dashboard.displayPrintf(lineNum, "Target=%.1f, Input=%.1f, Error=%.1f", setPoint, pidInput.getInput(this), prevError);
        //dashboard.displayPrintf(lineNum + 1, "minOutput=%.1f, Output=%.1f, maxOutput=%.1f", minOutput, output, maxOutput);
        System.out.printf("Target=%.1f, Input=%.1f, Error=%.1f", setPoint, pidInput.getInput(this), prevError);
        System.out.println();
        System.out.printf("minOutput=%.1f, Output=%.1f, maxOutput=%.1f", minOutput, output, maxOutput);
    }

    public void setTargetRange(double minTarget, double maxTarget)
    {
        this.minTarget = minTarget;
        this.maxTarget = maxTarget;
    }

    //Inverts information if needed
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public void setAbsoluteSetPoint(boolean absolute)
    {
        this.absSetPoint = absolute;
    }

    public void setPID(double kP, double kI, double kD, double kF)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    //Sets distance in ticks
    public void setTarget(double target)
    {
        double input = pidInput.getInput(this);
        if (!absSetPoint)
        {
            //
            // Set point is relative, add target to current input to get absolute set point.
            //
            setPoint = input + target;
            prevError = target;
            displayPidInfo(0);
        }
        else
        {
            //
            // Set point is absolute, use as is.
            //
            setPoint = target;
            prevError = setPoint - input;
            displayPidInfo(0);
        }
        setPointSign = Math.signum(prevError);
        //
        // If there is a valid target range, limit the set point to this range.
        //
        if (maxTarget > minTarget)
        {
            if (setPoint > maxTarget)
            {
                setPoint = maxTarget;
            }
            else if (setPoint < minTarget)
            {
                setPoint = minTarget;
            }
            displayPidInfo(0);
        }

        prevTime = HalUtil.getCurrentTime();
        if (inverted)
        {
            prevError = -prevError;
        }
        totalError = 0.0;
        settlingStartTime = HalUtil.getCurrentTime();
    }

    //Returns previous error
    public double getError() {
        return prevError;
    }

    public void reset()
    {
        prevError = 0.0;
        prevTime = 0.0;
        totalError = 0.0;
        setPoint = 0.0;
        setPointSign = 1.0;
        output = 0.0;
    }

    //Determines whether or not the robot is on track
    public boolean isOnTarget()
    {
        boolean onTarget = false;

        if (noOscillation)
        {
            //
            // Don't allow oscillation, so if we are within tolerance or we pass target, just quit.
            //
            if (prevError*setPointSign <= tolerance)
            {
                onTarget = true;
            }
        }
        else if (Math.abs(prevError) > tolerance)
        {
            settlingStartTime = HalUtil.getCurrentTime();
        }
        else if (HalUtil.getCurrentTime() >= settlingStartTime + settlingTime)
        {
            onTarget = true;
        }

        return onTarget;
    }

    public void setOutputRange(double minOutput, double maxOutput)
    {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }   //setOutputRange

    //Calculates the power output for driving forward
    public double getPowerOutput()
    {
        double currTime = HalUtil.getCurrentTime();
        double deltaTime = currTime - prevTime;
        prevTime = currTime;
        input = pidInput.getInput(this);
        double error = setPoint - input;
        if (inverted)
        {
            error = -error;
        }

        if (kI != 0.0)
        {
            //
            // Make sure the total error doesn't get wound up too much exceeding maxOutput.
            //
            double potentialGain = (totalError + error * deltaTime) * kI;
            if (potentialGain >= maxOutput)
            {
                totalError = maxOutput / kI;
            }
            else if (potentialGain > minOutput)
            {
                totalError += error * deltaTime;
            }
            else
            {
                totalError = minOutput / kI;
            }
        }

        pTerm = kP*error;
        iTerm = kI*totalError;
        dTerm = deltaTime > 0.0 ? kD*(error - prevError)/deltaTime: 0.0;
        fTerm = kF*setPoint;
        output = fTerm + pTerm + iTerm + dTerm;

        prevError = error;
        if (output > maxOutput)
        {
            output = maxOutput;
        }
        else if (output < minOutput)
        {
            output = minOutput;
        }

        return output;
    }

}