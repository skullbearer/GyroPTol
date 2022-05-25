
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;


namespace IngameScript
{

    class GyroPTol
    { // This class contains P-gain loops with or without self-tuning, self-adjusting, and/or error tolerance for gyro controlled grids without subgrids/connected grids involved. Will not work reliably with real-thruster physics mods.
      // Each axis will have it's own gains and tolerance
      // Thank you to Whiplash for having an awesome PID script class already!

        ////////////////////////// - Intro - //////////////////////////////////
        // GyroPTol class variable
        // Written by Skullbearer
        //
        ////////////////////////// - End Intro - //////////////////////////////


        ////////////////////////// - Instructions - //////////////////////////////
        // This has been setup in reverse to Whip's PID setup ON PURPOSE!
        // Why? WHY DO THAT SKULLBEARER???
        //
        // Because they are different and I want to make you actually intentionally
        // switch the code and not just copy-paste swap our PID classes. I'm not
        // making it directly interchangeable as I don't want to update-lock to Whip's.
        // 
        // Why update the PID that Whiplash uses? Frankly too many people complain and don't
        // fully understand the limits or capacities of PID. Also, frankly, this is Space Engineers
        // and not a real world problem. We don't need I or D gain in a tuned system. You can
        // use this to pull out tuned P gains and then plug them right into Whip's PID class methods
        // without any issue, which is totally fine. If you need D gain, your P gain isn't properly
        // tuned. If you need I gain, you have sub-grids, connected grids, or a real thruster
        // physics mod that is causing you to turn under acceleration (ie gravity, or thrust).
        //
        // For that, you'll need my other class which isn't publicshed at the time of this writing.
        //
        // If you are flying a missile: WOBBLE IS FINE, you just want very little wobble.
        // IN FACT!!!! Wobble means it's tracking around the target vector perfectly just correcting
        // back and forth. Update10 is MORE THAN ENOUGH and if you really needed it like on a high
        // speed server setting with extreme thrust, you could shift to Update1 ONLY JUST BEFORE IMPACT
        //
        // If you are flying a drone or ship: It can be legit that you don't want wobble.
        // Particularly for long trips, the constant wobble will drain power and also can induce
        // motion sickness. That's totally fine! In that case, we allow an error tolerance! Literally
        // we just say that if we're 'close enough' then we stop trying to aim better. When the aim
        // drifts too much (like as we get closer to out target GPS and the ship has been slightly
        // aimed wrong and now is outside our tolerance allowance of angle mismatch) then we correct
        // back to within the tolerance zone. This provides NO WOBBLE but is LESS ACCURATE.
        //
        // Summary:
        // - Missile or Very Short Flight Needing Maximum Accuracy? - errTolDeg = 0
        // - Drone or Long Flight Needing No Wobble? - errTolDeg > 0
        //
        //
        // How to Use?
        // This is just an example, you can use lists, individual floats for each axis, etc.
        //
        // GyroPTol xAxis = new GyroPTol();
        // GyroPTol yAxis = new GyroPTol();
        // GyroPTol zAxis = new GyroPTol();
        // float toAngle[] = [0,0,0]; // In radiians, x,y,z
        // float newCommand[] = [0,0,0]; // In rad/s, x,y,z
        // int numTicks = 10; // This is for Update10
        // float timeStep = numTicks/60.0;
        //
        // toAngle = yourStuffToDecideWhichAngleToTurn(); // Set all three axis
        // newCommand = [xAxis.PIDRun(toAngle[0], timeStep)  // This is the X-axis command for your gyros
        //               yAxis.PIDRun(toAngle[1], timeStep)  // This is the Y-axis command for your gyros
        //               zAxis.PIDRun(toAngle[2], timeStep)]; // This is the Z-axis command for your gyros
        // foreach(IMyGyro block in whateverYourGyroList<IMyGyro>IsCalled)
        // {
        //     //All your code to set the gyros at the correct axis
        // }
        // 
        // What do I need to set?
        //
        // Well, if you change them from vanilla defaults, then be sure to set:
        // - gyroPower
        // - maxRVel
        //
        //
        ////////////////////////// - End Instructions - //////////////////////////

        // P gain elements
        public double gainPA; // P gain for acceleration
        public double gainPD; // P gain for deceleration
        public double proportionalComponent;

        // I gain elements
        public double gainIA; // I gain for acceleration
        public double gainID; // I gain for deceleration
        double gainIHold;
        public double integralAComponent; // Integrated value, can be reset or decayed by host script.
        public double integralDComponent;

        // D gain elements
        public double gainDA; // D gain
        public double gainDD;
        double gainDHold;

        // J gain elements (none used, just tracked)
        public double jerkComponent;

        // Rotation velocity elements
        public double maxRVel;
        double maxRVelObserved;

        // Gyro power setting (directly scales torque)
        public double gyroPower;

        // Acceleration components
        public double minAcc;
        public double maxAcc;
        public double averageAcc;

        // Deceleration components
        public double minDec;
        public double maxDec;
        public double averageDec;

        // Angle tolerance for alignment pre-setting, overridable on a per-method basis
        public double errTolDeg; // Error tolerance in degrees for easy user visualization

        // Absolute rotation elements
        public double rAcc; // Angular acceleration (rad/s/s), same as derivative component
        public double rVel; // Angular velocity (rad/s)
        public double lastRAcc; // Angular acceleration the last calculation time (rad/s/s)
        public double lastRVel; // Angular velocidy (rad/s)

        // Time elements
        double timeStepMin = 1.0 / 60.0; // This is the default for 1 tick

        // Output tracker for user to error check
        public double lastPIDout;
        public double lastAngCommand;

        // Self-adjustement elements
        public bool goodAverage;
        public int averageCnt;
        public int averageCntTarget;
        public double timeAverage;
        bool firstStep;
        public bool axisIsTuned;
        bool reverseIt;


        public GyroPTol()
        { // Constructor
            Reset();
            firstStep = true;
        }

        public void Reset()
        { // Set to defaults
            gainPA = gainPD = 0;
            proportionalComponent = 0;
            gainIA = gainID = 0;
            integralAComponent = 0;
            integralDComponent = 0;
            gainDA = gainDD = 0;
            jerkComponent = 0;
            maxRVel = Math.PI; // Vanilla gyros have a Math.Pi/s maximum angular velocity hardcoded into the game
            maxRVelObserved = 0;
            gyroPower = 1; // Assume 100% power until the use says otherwise.
            minAcc = 0;
            maxAcc = 0;
            averageAcc = 0.1;
            minDec = 0;
            maxDec = 0;
            averageDec = 0.1;
            errTolDeg = 0;
            rAcc = 0;
            rVel = 0;
            lastRAcc = 0;
            lastRVel = 0;


            lastPIDout = 0;
            lastAngCommand = 0;
            goodAverage = false;
            averageCnt = 0;
            averageCntTarget = 60; // 1s total time for a Update1, recommend at least 10.
            timeAverage = 1; // Seconds during which the average accelerations are averaged over

            axisIsTuned = false;
            reverseIt = false;
        }


        public double pTune(double angCommand, double angVel, double timeStep, double angTol = 0)
        { // This will seek to tune your P-gain loop! CAUTION! Perform this in open space! Returns the gainP value when complete.

            // We assume that the timeStep is an accurate time in seconds of game simulation time, which is almost impossible to tell for sure,
            // but if you're using Update10 and we're actually 11 ticks here and there, it's still going to provide a decent tune.

            // rAngle is the current angle on this axis (x, y, OR z, only 1 axis at a time) ins radiians, NOT DEGREES.

            // To perform this test, start with your grid at either 0 rotation speed or a slight negative speed on your tuning axis, such as -1RPM.
            // Command your gyros to all go to max speed, which is 30 RPM for vanilla gyros. Do this on ONE AXIS ONLY, the others should be 0 RPM.
            // Keep in mind that the UI is in RPM and the script interface is in radiians/s (rad/s), so the script command is for Math.PI.
            // In the same tick as starting the command, send the first call to this with your current angle rAngle and the expected timeStep
            // before the next call.
            //
            // Keep calling this function every timeStep until it returns a non-null value. You don't even have to check the value, it will be stored in gainP,
            // however if you check it, it's just the new gainP value.
            //
            // If there are no subgrids or real-thruster physics, then your loop is now tuned with gainI = gainD = 0;
            //
            // Continue to run until the axisIsTuned bool is true.
            //
            // The time it will take to tune and the amount your grid will rotate with depend on your timeStep * averageCntTarget and your grid's
            // acceleration into rotation. For a very large grid, a slow update rate and a high averageCntTarget is a good combination as well.

            double PIDout = 0;
            bool wasAccel;
            lastRAcc = rAcc;
            lastRVel = rVel;
            // Derivative function, updated rAcc
            rAcc = (angVel - rVel) / timeStep;
            // Update rVel to current.
            rVel = angVel;
            if (rVel > maxRVelObserved) maxRVelObserved = rVel;

            if (axisIsTuned || firstStep)
            {
                Reset();
            }


            if (lastRVel >= maxRVel - 0.01 || averageCnt >= averageCntTarget / 2)
            {
                reverseIt = true;
            }

            if (Math.Abs(rVel) < maxRVel && averageCnt < averageCntTarget)
            {
                PIDout = Math.PI;
                wasAccel = true;
                if (reverseIt) PIDout *= -1.0;
            }
            else
            {
                wasAccel = true;
                if (reverseIt && averageCnt > averageCntTarget)
                {
                    axisIsTuned = true;
                    reverseIt = false;
                    maxRVel = maxRVelObserved;
                    PIDout = 0;
                    wasAccel = false;
                }
                else if (reverseIt)
                { // If we accelerate just so fast that we have insufficient averaging data, repeat the accelerations.
                    reverseIt = false;
                }
            }

            pAdjustment(PIDout, wasAccel, timeStep);

            //if (firstStep) firstStep = false; // Now done inside pAdjustment()
            return lastPIDout = PIDout;
        }

        public double PIDRunSelfAdjusting(double angCommand, double angVel, double timeStep, double angTol = -1.0, bool isSelfAdjusting = true)
        {
            if (double.IsNaN(angCommand)) throw new Exception("angCommand is NaN");
            if (double.IsNaN(angVel)) throw new Exception("angVel is NaN");
            if (double.IsNaN(timeStep)) throw new Exception("timeStep is NaN");
            if (double.IsNaN(angTol)) throw new Exception("angTol is NaN");
            if (angCommand == 0.0) return lastPIDout = 0.0;
            if (timeStep < timeStepMin) throw new Exception("timeStep is smaller than the smallest possible value");
            if (double.IsNaN(gyroPower)) throw new Exception("gyroPower is NaN");
            if (double.IsNaN(gainPA)) throw new Exception("gainPA is NaN");
            if (double.IsNaN(gainPD)) throw new Exception("gainPD is NaN");
            if (double.IsNaN(gainIA)) throw new Exception("gainIA is NaN");
            if (double.IsNaN(gainID)) throw new Exception("gainID is NaN");
            if (double.IsNaN(gainDA)) throw new Exception("gainDA is NaN");
            if (double.IsNaN(gainDD)) throw new Exception("gainDD is NaN");
            if (double.IsNaN(rAcc)) throw new Exception("rAcc is NaN");
            if (double.IsNaN(rVel)) throw new Exception("rVel is NAN before update");

            double PIDout = 0.0;
            double rAccLast = rAcc;
            double timeToStop;
            double timeToTarget;
            double targetRVel;
            double errorVal;
            double lastErrorVal;

            bool isAcceleration;
            bool wasAcceleration;

            // Check if we were accelerating last command, for integral
            lastErrorVal = lastPIDout - lastRVel;
            if (Math.Sign(lastErrorVal) == Math.Sign(lastAngCommand)) wasAcceleration = true;
            else wasAcceleration = false;

            // Update rVel to current.
            lastRVel = rVel;
            rVel = angVel;

            // Our self adjustment is based in part on maxRVel, make sure we catch it correctly
            if (Math.Abs(rVel) > maxRVel) maxRVel = Math.Abs(rVel);
            if (double.IsNaN(maxRVel)) throw new Exception("maxRVel is NaN");

            if (angTol == -1.0) angTol = errTolDeg; // If it's never set, then use the stored value.

            // errTolDeg cannot ever be less than zero, that is already perfect accuracy.
            if (angTol < 0.0) angTol = 0.0;

            // Derivative function, updated rAcc
            lastRAcc = rAcc;
            rAcc = (angVel - lastRVel) / timeStep;
            if (wasAcceleration)
            {
                if (Math.Abs(rAcc) > maxAcc) maxAcc = Math.Abs(rAcc);
                else if (maxAcc < 0) maxAcc = Math.Abs(maxAcc);
                if (Math.Abs(lastPIDout) > Math.PI - 0.01 && (Math.Abs(rAcc) < minAcc || minAcc == 0)) minAcc = Math.Abs(rAcc);
            }
            else
            {
                if (Math.Abs(rAcc) > maxDec) maxAcc = Math.Abs(rAcc);
                else if (maxAcc < 0) maxDec = Math.Abs(maxDec);
                if (Math.Abs(lastPIDout) > Math.PI - 0.01 && (Math.Abs(rAcc) < minDec || minDec == 0)) minDec = Math.Abs(rAcc);
            }
            if (!double.IsNaN(averageDec) && averageDec != 0.0) // If we have a real averageDec
                timeToStop = Math.Abs(rVel / averageDec);
            else if (!double.IsNaN(averageAcc) && averageAcc != 0.0) // If we don't, but we have averageAcc
                timeToStop = Math.Abs(rVel / averageAcc);
            else timeToStop = 0; // If we have no acceleration averages then just make it 0 to make things move.
            if (Math.Abs(angVel) > 0.0) // If we're moving
                timeToTarget = Math.Abs(angCommand / angVel);
            else timeToTarget = 3600; // Otherwise set an arbitrary long time (1 hour)
            // Track a target velocity
            if (angVel != 0.0 && timeToTarget > timeToStop + timeStep)
            {
                targetRVel = Math.Max(maxRVel, Math.PI);
            }
            else if (timeToTarget < timeToStop)
                targetRVel = 0;
            else
                targetRVel = Math.Min(maxRVel, averageDec * timeToTarget);



            errorVal = targetRVel * Math.Sign(angCommand) - rVel;
            if (double.IsNaN(errorVal)) throw new Exception("errorVal is NaN");

            if (Math.Sign(errorVal) == Math.Sign(angCommand)) isAcceleration = true;
            else isAcceleration = false;

            // Integrator function
            if (wasAcceleration)
            { // Integrate on acceleration
                integralAComponent += errorVal * timeStep;
                if (double.IsNaN(integralAComponent)) throw new Exception("integralAComponent is NaN");
            }
            else
            { // Integrate on deceleration
                integralDComponent += errorVal * timeStep;
                if (double.IsNaN(integralDComponent)) throw new Exception("integralDComponent is NaN");
            }

            // Adjust for remaining error not accounted for last step by adjusting the P gain for that.
            if (isSelfAdjusting)
            {
                pAdjustment(targetRVel, wasAcceleration, timeStep);
                // If it was the first step, we threw out the first acceleration data point
                //if (firstStep) firstStep = false; // now done inside pAdjustment()
            }

            // PID+Feed Forward equation, complete. Outputs a target velocity command to the gyro adjusted for gainP.
            if (isAcceleration)
                PIDout = gyroPower * ((gainPA * errorVal + targetRVel * Math.Sign(angCommand)) + gainIA * integralAComponent + gainDA * rAcc);
            
            else
                PIDout = gyroPower * ((gainPD * errorVal + targetRVel * Math.Sign(angCommand)) + gainID * integralDComponent + gainDD * rAcc);
            


            // Jerk calculation
            jerkComponent = (rAcc - rAccLast) / timeStep;

            if (angTol >= Math.Abs(angCommand)) return lastPIDout = 0.0;

            return lastPIDout = Math.Min(Math.Abs(PIDout), Math.PI) * Math.Sign(PIDout); // Preserves the sign of PIDout

        }

        void pAdjustment(double targetRVel, bool wasAccel, double timeStep)
        {
            //double timeToStop = 0;
            // Derive the maximum P-gain to not grossly overshoot
            if (((wasAccel && Math.Abs(lastPIDout) > maxRVel - 0.001) || (!wasAccel && Math.Abs(lastPIDout) < 0.001)) && rVel != lastPIDout && Math.Abs(rAcc) > 0 && !goodAverage)
            {
                if (!firstStep)
                { // skip the first step, no acceleration data exists
                    applyAverage(wasAccel, timeStep);
                    if (averageCnt > averageCntTarget) goodAverage = true;
                    averageCnt++;
                }
                else firstStep = false;
            }
            if (Math.Abs(lastPIDout) > maxRVel - 0.001 && rVel != lastPIDout && Math.Abs(rAcc) > 0 && goodAverage)
            { // Until testing shows otherwise, torque seems to scale with the command as well as power

                // This is only calculated when we know we were accelerating with the maximum command
                // and we did not achieve the target speed and so we captured only full acceleration
                // otherwise we just return the normal PIDout without modifying the gains

                applyAverage(wasAccel, timeStep);

            }
            if (goodAverage && averageAcc > 0 && averageDec > 0)
            { // Attempts to dynamically correct for command tracking errors, should account for damping effects (roughly)
                if (wasAccel)
                    gainPA = (targetRVel - Math.Abs(rVel)) * lastRAcc / averageAcc;
                else
                    gainPD = (targetRVel - Math.Abs(rVel)) * lastRAcc / averageDec;
            }
            return;
        }

        void applyAverage(bool wasAccel, double timeStep)
        {
            if (wasAccel)
                averageAcc = (averageAcc * timeAverage + Math.Abs(rAcc) * timeStep) / (timeAverage + timeStep);
            else
                averageDec = (averageDec * timeAverage + Math.Abs(rAcc) * timeStep) / (timeAverage + timeStep);
        }
    }

}