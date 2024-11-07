package org.firstinspires.ftc.teamcode.system.accessory.pids;

import com.qualcomm.robotcore.util.ElapsedTime;
public class FullStateFeedback
{
    /*
    This type of controller will have much better trajectory tracking performance than pid as in addition
    to driving our robot to the target position, it is able to drive our robot at target velocity.
    In combination with motion profiling this is an incredibly powerful tool.
    You can even extend this method by adding additional states.
    Combined with feedforward control you will have an incredibly robust controller.
     */

    ElapsedTime timer = new ElapsedTime();
    double k1,k2;
    double vel, prevPos;
    /**@param k1 Positional gain
     * @param k2 Velocity gain*/
    public FullStateFeedback(double k1, double k2)
    {
        this.k1 = k1; // Positional gain
        this.k2 = k2; // Velocity gain
    }
    public double update(double target, double position, double targetVel)
    {
        vel = (position - prevPos) / timer.seconds();
        double error = target - position;
        double errorVel = targetVel - vel;
        double update = (error * k1) + (errorVel * k2);

        this.prevPos = position;
        timer.reset();
        return update;

    }

    public double updateWithError(double error, double position, double targetVel)
    {
        vel = (position - prevPos) / timer.seconds();
        double errorVel = targetVel - vel;
        double update = (error * k1) + (errorVel * k2);

        this.prevPos = position;
        timer.reset();
        return update;

    }
    public void setGains(double k1, double k2)
    {
        this.k1 = k1; // Positional gain
        this.k2 = k2; // Velocity gain
    }

}
