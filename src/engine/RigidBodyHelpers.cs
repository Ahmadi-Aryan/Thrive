using Godot;
using System;
using System.Diagnostics;

/// <summary>
///   Common helper operations for RigidBodies
/// </summary>
public static class RigidBodyHelpers
{
    /// <summary>
    /// Basic rotation physics for a RigidBody based on a target point
    /// Not optimized :p
    public static void LookFollow(this RigidBody body, PhysicsDirectBodyState state,
        Transform transform, Vector3 targetPoint, float RotationSpeed)
    {
        // various directions
        var up = new Vector3(0, 1, 0);
        var direction = transform.basis.Quat().Normalized();
        var targetDirection = transform.LookingAt(targetPoint, up).basis.Quat().Normalized();

        // quaternion representing desired rotation and axis angle pair
        var rotDifference = (targetDirection * direction.Inverse()).Normalized();
        var rotAxis = rotDifference.GetEuler().Normalized();
        // [-pi,+pi]
        var rotAngle = rotDifference.GetEuler().y;



        //var acc = RotationTorque * Mathf.Pow(body.Mass, 2);
        //var dcc = -TorqueDrag * Mathf.Pow(body.Mass, 2);


        // Crashes if inverting with Det() = 0
        var inertia = 1.0f;
        if (body.GetInverseInertiaTensor().Determinant() != 0)
        {
            //Should be a Microbe parameter instead of being calculated every tick (duh)
            inertia = body.GetInverseInertiaTensor().Inverse().y.y;
        }


        var RotationTorque = 50.0f * RotationSpeed;

        // Multiply Torque by Inertia to get constant acceleration for a given rotation speed
        // Rotation speed should be rebalanced, seems to scale too easily especially for large cells.
        // Ideally Drag should depend on shape & other factors

        var TorqueDrag = 1.0f;

        var acc = RotationTorque * inertia;
        var dcc = -TorqueDrag * inertia;

        // Angular velocity [-/+]
        var wspeed = state.AngularVelocity.y;


        // eta is a rough estimate for determining when it should brake
        float dampingfactor = 2.0f;
        float eta = rotAngle - dampingfactor * (wspeed/(RotationTorque+TorqueDrag) * (0.5f + Mathf.Abs(wspeed)));
        // last term helps get rid of the wobble at low angles while being effective at high speeds

        // optional - tldr this should take into account current speed when turning at large angles.
        // can most likely be written better
        if (eta > 2 * Mathf.Pi)
        {
            eta -= 4 * Mathf.Pi;
        }
        else if (eta < -2 * Mathf.Pi)
        {
            eta += 4 * Mathf.Pi;
        }

        // Logistics Curve for "throttle"
        // Higher Steepness makes it more abrupt
        float steepness = 10.0f;
        float scurve = 2 / (1 + Mathf.Exp(-steepness * eta)) - 1;

        // Cell Acceleration / Decelartion
        body.AddTorque(rotAxis.Abs() * scurve * acc);

        // Drag Deceleration
        // Could also scale with square of speed (would help keep rotation speeds more balanced) but at slow speeds drag is proportional to speed
        // Inbuilt Angular Damping Could possibly be used instead
        body.AddTorque(state.AngularVelocity * dcc);

        //Something more akin to this could be implemented for movement (in particular shape should affect max speed, along with possibly other factors)

    }

    /// <summary>
    ///   Just slerps by a fixed amount towards the target point.
    ///   Weight of the slerp determines turn speed. Collisions dodgy.
    /// </summary>
    public static Transform LookSlerp(this RigidBody body, PhysicsDirectBodyState state, Vector3 targetPoint)
    {
        Transform target = state.Transform.LookingAt(targetPoint, new Vector3(0, 1, 0));

        // Need to manually normalize everything, otherwise the slerp fails
        Quat direction = state.Transform.basis.Quat().Normalized();
        Quat targetDirection = direction.Slerp(target.basis.Quat().Normalized(), 0.2f);

        return new Transform(new Basis(targetDirection), state.Transform.origin);
    }
}