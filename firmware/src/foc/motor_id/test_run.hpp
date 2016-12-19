/**
 * Copyright (c) 2016  Zubax Robotics OU  <info@zubax.com>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "common.hpp"


namespace foc
{
namespace motor_id
{
/**
 * Simple rotor stall detection logic
 */
class TestRunStallDetector
{
    const MotorParameters m_;

    math::FivePointDifferentiator<Scalar> Iq_gradient_estimator_;
    math::FivePointDifferentiator<Scalar> vp_estimator_;
    math::FivePointDifferentiator<Scalar> vp2_estimator_;
    math::SampleVariance<Scalar> vp2_variance_estimator_;

    Scalar v_ = 0;
    Scalar vp_ = 0;
    Scalar vp2_ = 0;
    Scalar vp2_stdev_ = 0;

    Scalar remaining_suppression_time_ = 0;
    bool suppressed_ = false;

    bool stall_ = false;

    variable_tracer::ProbeGroup<3> probes_;

public:
    // trace -c time AVel sdA2 sdAs sdSD
    TestRunStallDetector(const MotorParameters& motor_params) :
        m_(motor_params),
        probes_("sdA2", &vp2_,
                "sdAs", &vp2_stdev_,
                "sdSD", &stall_)
    { }

    void update(Const period, const Vector<2>& Idq, const Vector<2>& Udq)
    {
        if (suppressed_)
        {
            remaining_suppression_time_ -= period;
            if (remaining_suppression_time_ <= 0.0F && !stall_)
            {
                suppressed_ = false;
            }
        }

        Const lowpass_innovation = period * 10.0F;

        // Ang vel update
        // TODO Ask Dmitry to document the theory behind this equation
        Const Iq_prime = Iq_gradient_estimator_.update(Idq[1]) / period;
        Const new_output = std::abs((Udq[1] - m_.rs * Idq[1] - m_.lq * Iq_prime) / (m_.lq * Idq[0] + m_.phi));
        v_ += lowpass_innovation * (new_output - v_);

        // Ang vel derivatives update
        vp_ += lowpass_innovation * (vp_estimator_.update(v_) / period - vp_);

        vp2_ += lowpass_innovation * (vp2_estimator_.update(vp_) / period - vp2_);

        // Stall detection BEFORE statistics is updated
        if (vp2_variance_estimator_.getNumSamples() > 10000)
        {
            // Assuming that v'' has zero mean - for the second derivative it should hold
            stall_ = std::abs(vp2_) > (vp2_stdev_ * 6.0F);     // TODO: configurable
        }
        else
        {
            stall_ = false;
        }

        // Statistics update must be performed AFTER stall detection
        vp2_variance_estimator_.addSample(vp2_);
        if (vp2_variance_estimator_.areEstimatesAvailable())
        {
            vp2_stdev_ = vp2_variance_estimator_.getStandardDeviation();
        }
    }

    void expectFalsePositiveAhead(Const eta = 1.0F)
    {
        remaining_suppression_time_ = eta;
        suppressed_ = true;
    }

    bool isStallDetected() const
    {
        return suppressed_ ? false : stall_;
    }

    void reset()
    {
        Iq_gradient_estimator_.reset();
        vp_estimator_.reset();
        vp2_estimator_.reset();
        vp2_variance_estimator_.reset();

        v_          = 0;
        vp_         = 0;
        vp2_        = 0;
        vp2_stdev_  = 0;

        remaining_suppression_time_ = 0;
        suppressed_ = false;
        stall_ = false;
    }
};

/**
 * This task spins the motor in open loop under various conditions.
 * The variables observed during its execution can be used to identify parameters of the motor off-line.
 * Variables can be observed and recorded in real time using the existing variable tracing facility.
 */
class TestRunTask : public ISubTask
{
    static constexpr Scalar MaxElectricalAngularVelocity = 4500.0F;     ///< [rad/sec]

    /**
     * Note that we're using default settings of the three phase modulator.
     * It is important to use same settings that are used in normal operating mode,
     * otherwise model identification/adjustment may be rendered invalid.
     */
    using Modulator = ThreePhaseVoltageModulator<>;

    SubTaskContextReference context_;
    const MotorParameters result_;

    Const min_stable_angular_velocity_;

    Scalar remaining_delay_ = 0;

    Scalar angular_velocity_ = 0;
    Scalar angular_acceleration_ = 0;

    Modulator modulator_;
    Modulator::Setpoint setpoint_;
    Modulator::Output latest_modulator_output_;

    TestRunStallDetector stall_detector_;

    Status status_ = Status::InProgress;

    const variable_tracer::ProbeGroup<5> probes_;


public:
    TestRunTask(SubTaskContextReference context, const MotorParameters& result) :
        context_(context),
        result_(result),
        min_stable_angular_velocity_(result.min_electrical_ang_vel * 2.0F),

        modulator_(result.lq,
                   result.rs,
                   result.max_current,
                   context.params.controller.voltage_modulator_bandwidth,
                   context.board.pwm,
                   Modulator::DeadTimeCompensationPolicy::Disabled,
                   Modulator::CrossCouplingCompensationPolicy::Disabled),

        setpoint_{0.0F, Modulator::Setpoint::Mode::Iq},

        stall_detector_(result),

        probes_("Ud",   &latest_modulator_output_.reference_Udq[0],
                "Uq",   &latest_modulator_output_.reference_Udq[1],
                "Id",   &latest_modulator_output_.Idq[0],
                "Iq",   &latest_modulator_output_.Idq[1],
                "AVel", &angular_velocity_)
    {
        // Model is required for this task
        if (!result_.isValid())
        {
            status_ = Status::Failed;
        }
    }

    void onMainIRQ(Const period, const board::motor::Status&) override
    {
        if (status_ != Status::InProgress)
        {
            return;
        }

        if (remaining_delay_ > 0)
        {
            remaining_delay_ -= period;
            return;
        }

        Vector<2> Idq;
        Vector<2> Udq;

        {
            AbsoluteCriticalSectionLocker locker;
            Idq = latest_modulator_output_.Idq;
            Udq = latest_modulator_output_.reference_Udq;
        }

        stall_detector_.update(period, Idq, Udq);

        if (stall_detector_.isStallDetected())
        {
            status_ = Status::Failed;
        }

        if (angular_velocity_ > 4000.0F)
        {
            status_ = Status::Succeeded;
        }
        else
        {
            angular_acceleration_ = 100.0F;

            if (angular_velocity_ < 1000.0F)
            {
                setpoint_.value = 0.5F;
            }
            else if (angular_velocity_ < 2000.0F)
            {
                if (setpoint_.value < 3.9F)
                {
                    stall_detector_.expectFalsePositiveAhead();
                    setpoint_.value = 4.0F;
                }
            }
            else if (angular_velocity_ < 3000.0F)
            {
                if (setpoint_.value < 9.9F)
                {
                    stall_detector_.expectFalsePositiveAhead();
                    setpoint_.value = 10.0F;
                }
            }
            else
            {
                setpoint_.value = 1.0F;
            }
        }

        // Angular velocity and setpoint update
        setpoint_.value = std::copysign(setpoint_.value, angular_velocity_);
        angular_velocity_ += angular_acceleration_ * period;
    }

    void onNextPWMPeriod(const Vector<2>& phase_currents_ab, Const inverter_voltage) override
    {
        if (status_ != Status::InProgress)
        {
            return;
        }

        if (remaining_delay_ > 0)
        {
            latest_modulator_output_ = Modulator::Output();
            return;
        }

        latest_modulator_output_ =
            modulator_.onNextPWMPeriod(phase_currents_ab,
                                       inverter_voltage,
                                       angular_velocity_,
                                       latest_modulator_output_.extrapolated_angular_position,
                                       setpoint_);

        context_.setPWM(latest_modulator_output_.pwm_setpoint);
    }

    Status getStatus() const override { return status_; }

    MotorParameters getEstimatedMotorParameters() const override { return result_; }
};

}
}
