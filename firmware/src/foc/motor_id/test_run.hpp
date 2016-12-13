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
 * This task spins the motor in open loop under various conditions.
 * The variables observed during its execution can be used to identify parameters of the motor off-line.
 * Variables can be observed and recorded in real time using the existing variable tracing facility.
 */
class TestRunTask : public ISubTask
{
    /**
     * Note that we're using default settings of the three phase modulator.
     * It is important to use same settings that are used in normal operating mode,
     * otherwise model identification/adjustment may be rendered invalid.
     */
    using Modulator = ThreePhaseVoltageModulator<>;

    class StallDetector
    {
        math::RollingSampleVariance<500, Scalar> variance_estimator_;
        math::SimpleMovingAverageFilter<500, Scalar> I_prefilter_;
        Scalar I_filtered_ = 0;

    public:
        StallDetector() :
            I_prefilter_(0.0F)
        { }

        bool update(Const period, const Vector<2> Idq)
        {
            Const prev_I = I_filtered_;

            Const innovation = period * 10.0F;
            I_prefilter_.update(Idq.norm());
            I_filtered_ += innovation * (I_prefilter_.getValue() - I_filtered_);

            Const dIdt = (I_filtered_ - prev_I) / period;

            // TODO: Proper threshold estimation!
            Const dIdt_threshold = 400.0F;

            return std::abs(dIdt) > dIdt_threshold;
        }

        void reset()
        {
            I_filtered_ = 0.0F;
            I_prefilter_.reset(0.0F);
        }
    } stall_detector_;

    SubTaskContextReference context_;
    const MotorParameters result_;

    Scalar angular_velocity_ = 0;
    Scalar angular_acceleration_ = 0;

    Modulator modulator_;
    Modulator::Setpoint setpoint_;
    Modulator::Output latest_modulator_output_;

    Status status_ = Status::InProgress;

    /// These probes allow the outside world to view the variables of interest in real time
    const variable_tracer::ProbeGroup<5> probes_;

public:
    TestRunTask(SubTaskContextReference context, const MotorParameters& result) :
        context_(context),
        result_(result),

        modulator_(result.lq,
                   result.rs,
                   result.max_current,
                   context.params.controller.voltage_modulator_bandwidth,
                   context.board.pwm,
                   Modulator::DeadTimeCompensationPolicy::Disabled,
                   Modulator::CrossCouplingCompensationPolicy::Disabled),

        probes_("Ud",   &latest_modulator_output_.reference_Udq[0],
                "Uq",   &latest_modulator_output_.reference_Udq[1],
                "Id",   &latest_modulator_output_.Idq[0],
                "Iq",   &latest_modulator_output_.Idq[1],
                "AVel", &angular_velocity_)
    {
        // TODO this is temporary
        angular_acceleration_ = 50.0F;
        setpoint_.mode = Modulator::Setpoint::Mode::Iq;
        setpoint_.value = 4.0F;
    }

    void onMainIRQ(Const period, const board::motor::Status&) override
    {
        if (status_ != Status::InProgress)
        {
            return;
        }

        // TODO this is temporary
        {
            AbsoluteCriticalSectionLocker locker;   // Accessing shared data, lock is required

            if (stall_detector_.update(period, latest_modulator_output_.Idq))
            {
                status_ = Status::Failed;
            }
        }

        // TODO this is temporary
        if (angular_velocity_ > 1000.0F)
        {
            angular_acceleration_ *= -1.0F;
        }
        if (angular_velocity_ < -1000.0F)
        {
            status_ = Status::Succeeded;
        }

        setpoint_.value = std::copysign(std::max(0.1F, std::abs(angular_velocity_) * 0.01F), angular_velocity_);

        angular_velocity_ += angular_acceleration_ * period;
    }

    void onNextPWMPeriod(const Vector<2>& phase_currents_ab, Const inverter_voltage) override
    {
        if (status_ != Status::InProgress)
        {
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
