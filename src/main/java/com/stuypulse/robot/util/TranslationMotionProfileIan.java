
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class TranslationMotionProfileIan {

    // Default number of times to apply filter (helps accuracy)
    private static final int kDefaultSteps = 64;

    // Stopwatch to Track dt
    private Timer mTimer;

    // Limits for each of the derivatives
    private Number mVelLimit;
    private Number mAccelLimit;

    // The last output / velocity
    private Translation2d mOutput;
    private Translation2d mVelocity;

    // Number of times to apply filter (helps accuracy)
    private final int mSteps;

    public TranslationMotionProfileIan(
        Number velLimit, 
        Number accelLimit, 
        Translation2d startingTranslation, 
        Translation2d startingVelocity, 
        int steps) 
    {
        mTimer = new Timer();
        mTimer.reset();

        mVelLimit = velLimit;
        mAccelLimit = accelLimit;

        mOutput = startingTranslation;
        mVelocity = startingVelocity;

        mSteps = steps;
    }

    public TranslationMotionProfileIan(Number velLimit, Number accelLimit, Translation2d startingTranslation, Translation2d startingVelocity) {
        this(velLimit, accelLimit, startingTranslation, startingVelocity, kDefaultSteps);
    }

    public TranslationMotionProfileIan(Number velLimit, Number accelLimit) {
        this(velLimit, accelLimit, new Translation2d(), new Translation2d(), kDefaultSteps);
    }

    public Translation2d get(Translation2d target) {
        double dt = mTimer.get() / mSteps;
        mTimer.reset();

        for (int i = 0; i < mSteps; ++i) {
            // if there is a accel limit, limit the amount the velocity can change
            if (0 < mAccelLimit.doubleValue()) {
                // amount of windup in system (how long it would take to slow down)
                double windup = mVelocity.getDistance(new Translation2d()) / mAccelLimit.doubleValue();

                Translation2d accel = new Translation2d();
                // If the windup is too small, just use normal algorithm to limit acceleration
                if (windup < dt) {
                    // Calculate acceleration needed to reach target
                    accel = target.minus(mOutput).div(dt).minus(mVelocity);
                } else {
                    // the position it would end up if it attempted to come to a full stop
                    Translation2d windA =
                        mVelocity.times(0.5 * (dt + windup)); // windup caused by acceleration
                    
                    Translation2d future = mOutput.plus(windA); // where the robot will end up

                    // Calculate acceleration needed to come to stop at target throughout windup
                    accel = target.minus(future).div(windup);

                }

                //Clamping the value between 0 and the maximum acceleration within dt time
                double maxMag = dt*mAccelLimit.doubleValue();
                if(maxMag<=0) accel = new Translation2d();
                else if(maxMag < accel.getDistance(new Translation2d())) accel.times(accel.getDistance(new Translation2d())/maxMag);
                
                // Try to reach it while abiding by accel limit
                mVelocity = mVelocity.plus(accel);
                
            } else {
                // make the velocity the difference between target and current
                mVelocity = target.minus(mOutput).div(dt);
            }

            // if there is an velocity limit, limit the velocity
            if (0 < mVelLimit.doubleValue()) {
                double maxMag = mVelLimit.doubleValue();
                if(maxMag<=0) mVelocity = new Translation2d();
                else if(maxMag < mVelocity.getDistance(new Translation2d())) mVelocity=mVelocity.times(mVelocity.getDistance(new Translation2d())/maxMag);
            }

            Translation2d error = target.minus(mOutput);
            
            Translation2d unitError = (error.getDistance(new Translation2d()) <= 1e-9)? new Translation2d(1, 0): error.div(error.getDistance(new Translation2d()));

            double parallelMag = mVelocity.getX() * unitError.getX() + mVelocity.getY() * unitError.getY();
            Translation2d accelParallel = unitError.times(parallelMag);

            Translation2d accelPerpendicular = mVelocity.minus(accelParallel);

            double damping = Math.pow(0.5, dt);
            mVelocity = accelParallel.plus(accelPerpendicular.times(damping));

            // adjust output by calculated velocity
            mOutput = mOutput.plus(mVelocity.times(dt));
        }

        // Field.FIELD2D.getObject("Translation Motion Profile Ian").setPose(!Robot.isBlue()
        //     ? new Pose2d(mOutput.x, mOutput.y, new Rotation2d())
        //     : Field.transformToOppositeAlliance(new Pose2d(mOutput.x, mOutput.y, new Rotation2d())));
            
        return mOutput;
    }
}
