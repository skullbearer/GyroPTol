
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

        double angSpd;
        double rAccAbs;
        double lastPIDAbs;
        double cmndDir;

        // Time elements
        double timeStepMin = 1.0 / 60.0; // This is the default for 1 tick
        double timeToReachTarget;

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
            gainPA = gainPD = 0d;
            proportionalComponent = 0d;
            gainIA = gainID = 0d;
            integralAComponent = 0d;
            integralDComponent = 0d;
            gainDA = gainDD = 0d;
            jerkComponent = 0d;
            gyroPower = 1.0; // Assume 100% power until the user says otherwise.
            averageAcc = 12.0;
            averageDec = 12.0;
            errTolDeg = 0d;
            rAcc = 0d;
            rVel = 0d;
            lastRAcc = 0d;
            lastRVel = 0d;


            lastPIDout = 0d;
            lastAngCommand = 0d;
            goodAverage = false;
            averageCnt = 0;
            averageCntTarget = 60; // 1s total time for a Update1, recommend at least 10.
            timeAverage = 1.0; // Seconds during which the average accelerations are averaged over

            axisIsTuned = false;
            reverseIt = false;
        }

        public double PIDRunSelfAdjusting(double _angCommand, double _angVel, double _timeStep, double _angTol = -1.0, bool _isSelfAdjusting = true)
        {
            if (double.IsNaN(_angCommand)) throw new Exception("angCommand is NaN");
            if (double.IsNaN(_angVel)) throw new Exception("angVel is NaN");
            if (double.IsNaN(_timeStep)) throw new Exception("timeStep is NaN");
            if (double.IsNaN(_angTol)) throw new Exception("angTol is NaN");
            if (_angCommand == 0.0) return lastPIDout = 0.0;
            if (_timeStep < timeStepMin) throw new Exception("timeStep is smaller than the smallest possible value");
            if (double.IsNaN(gyroPower)) throw new Exception("gyroPower is NaN");
            if (double.IsNaN(gainPA)) throw new Exception("gainPA is NaN");
            if (double.IsNaN(gainPD)) throw new Exception("gainPD is NaN");
            if (double.IsNaN(gainIA)) throw new Exception("gainIA is NaN");
            if (double.IsNaN(gainID)) throw new Exception("gainID is NaN");
            if (double.IsNaN(gainDA)) throw new Exception("gainDA is NaN");
            if (double.IsNaN(gainDD)) throw new Exception("gainDD is NaN");
            if (double.IsNaN(rAcc)) throw new Exception("rAcc is NaN");
            if (double.IsNaN(rVel)) throw new Exception("rVel is NAN before update");

            double _PIDout = 0.0;
            double _rAccLast = rAcc;
            double _timeToStop;
            double _timeToStopFuture;
            double _stopRatePerTick;
            double _timeToTarget;
            double _timeToTargetFwd;
            double _timeToTargetRev;
            double _targetRVel;
            double _errorVal;
            double _lastErrorVal;

            bool _isAcceleration;
            bool _wasAccel;
            bool _isFwd;

            angSpd = Math.Abs(_angVel);
            rAccAbs = Math.Abs(rAcc);
            lastPIDAbs = Math.Abs(lastPIDout);
            cmndDir = (double)Math.Sign(_angCommand);
            _stopRatePerTick = timeStepMin / averageDec;

            // Check if we were accelerating last command, for integral
            _lastErrorVal = lastPIDout - lastRVel;
            if (Math.Sign(_lastErrorVal) == Math.Sign(lastAngCommand)) _wasAccel = true;
            else _wasAccel = false;

            // Update rVel to current.
            lastRVel = rVel;
            rVel = _angVel;

            if (_angTol == -1.0) _angTol = errTolDeg; // If it's never set, then use the stored value.

            // errTolDeg cannot ever be less than zero, that is already perfect accuracy.
            if (_angTol < 0.0) _angTol = 0.0;

            // Derivative function, updated rAcc
            lastRAcc = rAcc;
            rAcc = (_angVel - lastRVel) / _timeStep;

            double _useAcc = averageAcc;
            double _useDec = averageDec;

            if (rVel > _stopRatePerTick)
            { // If we're moving faster than we can stop in just a single tick
                // If we were going to start braking now
                _timeToStop = Math.Abs(rVel / _useDec);
                // If we are going to accelerate now
                _timeToStopFuture = Math.Abs(rVel + _timeStep / _useAcc) / _useDec;
            }
            else if (rVel > 0.0001f)
            { // If we're moving and can stop in a single tick
                _timeToStop = timeStepMin;
                _timeToStopFuture = 0;
            }
            else // If we're not moving
            {
                _timeToStop = 0;
                _timeToStopFuture = 0;
            }

            timeToTarget(_angCommand, _angVel, out _timeToTargetFwd, out _timeToTargetRev);

            // Set class parameter in case the user wants it
            timeToReachTarget = Math.Min(_timeToTargetFwd, _timeToTargetRev);

            if (_timeToTargetFwd > _timeToTargetRev)
            { // Need to reverse direction!
                _isFwd = false;
            } // Don't reverse, keep on trucking!
            else
                _isFwd = true;

            if (_isFwd)
            {
                if (timeToReachTarget > _timeToStop + _timeStep)
                    _isAcceleration = true;
                else
                    _isAcceleration = false;
            }
            else // we're goint to stop, then go in reverse
                _isAcceleration = false;

            // Track a target velocity
            if (_isAcceleration)
                // Need to always set a speed slightly higher than we expect to achieve in order to maintain maximum accel
                _targetRVel = averageAcc * (_timeStep + timeStepMin);
            else if (_timeToStop < timeToReachTarget + _timeStep)
                // Need to always set a speed slightly lower than we expect to achieve in order to maintain maximum decel
                _targetRVel = averageDec * (_timeStep - timeStepMin);
            else// If we are coming to a final stop, don't be aggressive about it, go in by halves.
                _targetRVel = Math.Sign(_angCommand) / _timeStep / 2.0;

            _errorVal = _targetRVel * cmndDir - rVel;
            if (double.IsNaN(_errorVal)) throw new Exception("errorVal is NaN");

            // Integrator function
            if (_wasAccel)
            { // Integrate on acceleration
                integralAComponent += _errorVal * _timeStep;
                if (double.IsNaN(integralAComponent)) throw new Exception("integralAComponent is NaN");
            }
            else
            { // Integrate on deceleration
                integralDComponent += _errorVal * _timeStep;
                if (double.IsNaN(integralDComponent)) throw new Exception("integralDComponent is NaN");
            }

            // Adjust for remaining error not accounted for last step by adjusting the P gain for that.
            if (_isSelfAdjusting)
            { // Apply the averaging functions
                // Torque scales according to https://github.com/KeenSoftwareHouse/SpaceEngineers/blob/a109106fc0ded66bdd5da70e099646203c56550f/Sources/Sandbox.Game/Game/GameSystems/MyGridGyroSystem.cs
                if (rAccAbs > 0.001)
                {
                    if (rVel <= lastPIDout - _stopRatePerTick && !goodAverage)
                    { // If we hit our max acceleration last time...
                        if (!firstStep)
                        { // skip the first step, no acceleration data exists
                            applyAverage(_wasAccel, _timeStep);
                            if (averageCnt > averageCntTarget) goodAverage = true;
                            averageCnt++;
                        }
                        else firstStep = false;
                        gainPA = gainPD = 0.0;
                    }
                    else if (goodAverage)
                    {
                        // Specifically UpdateOverriddenGyros()
                        applyAverage(_wasAccel, _timeStep);
                        pAdjustment(_targetRVel, _wasAccel, _timeStep);
                    }
                    else // If we didn't hit our max acceleration
                    {
                        applyAverageIncrease(_wasAccel);
                        gainPA = gainPD = 0.0;
                    }
                }
            }

            // PID+Feed Forward equation, complete. Outputs a target velocity command to the gyro adjusted for gainP.
            if (_isAcceleration)
                _PIDout = ((gainPA * _errorVal + _targetRVel * cmndDir) + gainIA * integralAComponent + gainDA * rAcc);
            else
                _PIDout = ((gainPD * _errorVal + _targetRVel * cmndDir) + gainID * integralDComponent + gainDD * rAcc);

            // Jerk calculation
            jerkComponent = (rAcc - _rAccLast) / _timeStep;

            if (_angTol >= Math.Abs(_angCommand)) return lastPIDout = 0;

            return lastPIDout = Math.Min(Math.Abs(_PIDout), Math.PI) * Math.Sign(_PIDout); // Preserves the sign of PIDout

        }

        void pAdjustment(double _targetRVel, bool _wasAccel, double _timeStep)
        { // Adjust the micro-adjustment P-gain on prediction error
            // Attempts to dynamically correct for command tracking errors, should account for damping effects (roughly)
            if (_wasAccel)
                gainPA = (_targetRVel - Math.Abs(rVel)) * lastRAcc / averageAcc;
            else
                gainPD = (_targetRVel - Math.Abs(rVel)) * lastRAcc / averageDec;
            return;
        }

        void applyAverage(bool _wasAccel, double _timeStep)
        {
            double _k_prop;
            int _overrideAccelerationRampFrames;
            //double _torquePerc;

            _k_prop = (119.0 / (Math.PI * Math.PI / 4.0) + 1.0);
            _overrideAccelerationRampFrames = (int)(rVel * rVel * _k_prop) + 1;
            //_torquePerc = Math.Min((lastPIDout - lastRVel) * (60f / (float)_overrideAccelerationRampFrames), 1);
            // torque ~= (scalar) * (command - actual) / (command-actual)^2 ~= (scalar) / (command - actual) THEREFORE command ~= actual maximizes torque
            // There is a ceiling however, though it's not achievable on small and fast grids
            if (_wasAccel)
                averageAcc = (averageAcc * timeAverage + rAccAbs * _timeStep) / (timeAverage + _timeStep);
            else
                averageDec = (averageDec * timeAverage + rAccAbs * _timeStep) / (timeAverage + _timeStep);
        }

        void applyAverageIncrease(bool _wasAccel, double _scale = 1.05)
        { // Increases the calculated average rapidly until we're now averaging actual values
            if (_wasAccel)
                averageAcc *= _scale;
            else
                averageDec *= _scale;
        }

        void timeToTarget(double _angCmnd, double _angVel, out double _timeToTargetFwd, out double _timeToTargetRev)
        {
            double _accTimeFwd;
            double _accTimeRev;
            double _decTimeFwd;
            double _decTimeRev;
            double _accDistFwd;
            double _accDistRev;
            double _decDistFwd;
            double _decDistRev;

            _accTimeFwd = (Math.PI - Math.Abs(_angVel)) / averageAcc;
            _accTimeRev = Math.PI / averageAcc;
            _decTimeFwd = Math.PI / averageDec;
            _decTimeRev = Math.Abs(_angVel) / averageDec;
            _accDistFwd = _accTimeFwd * (Math.PI - Math.Abs(_angVel)) / 2.0;
            _accDistRev = _accTimeRev * Math.PI / 2.0;
            _decDistFwd = _decTimeFwd * Math.PI / 2.0;
            _decDistRev = _decTimeRev * Math.Abs(_angVel) / 2.0;

            if (_accDistFwd + _decDistFwd > Math.Abs(_angCmnd))
            { // If we'll never reach maximum speed during the continuation maneuver
                _timeToTargetFwd = Math.Abs(_angCmnd) / (_accDistFwd + _decDistFwd) * (_accTimeFwd + _accDistFwd);
            }
            else
            { // We would reach full speed during a continuation maneuver...
                _timeToTargetFwd = (Math.Abs(_angCmnd) - (_accDistFwd + _decDistFwd)) / Math.PI + (_accTimeFwd + _decTimeFwd);
            }
            if (_decDistFwd + _accDistRev > Math.Abs(_angCmnd))
            { // First we stop, then we go back, but don't hit max speed while doing so
                _timeToTargetRev = _decTimeRev + (_accTimeRev + _decTimeFwd) * (1 + _angCmnd / (_accDistRev + _decDistFwd));
            }
            else
            { // First we stop, then we go back, but hit max speed while doing so
                _timeToTargetRev = _decTimeRev + (_decDistRev + Math.Abs(_angCmnd) - (_accDistRev + _decDistFwd)) / Math.PI;
            }

        }
    }

}