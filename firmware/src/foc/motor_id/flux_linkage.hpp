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

class FluxLinkageTask : public ISubTask
{
    // Voltage reduction during the measurement phase shold be very slow in order to reduce phase delay of the filters.
    static constexpr Scalar VoltageSlopeLengthSec                       = 40.0F;
    static constexpr Scalar MinVoltage                                  = 0.001F;
    static constexpr unsigned MinSamplesBeforeStallDetectionEnabled     = 10000;
    static constexpr Scalar StallDetectionCurrentRateStdevMultiplier    = 5.0F;     // Make configurable?

    using Modulator = ThreePhaseVoltageModulator<>;

    SubTaskContextReference context_;
    MotorParameters result_;

    Const initial_Uq_;

    Scalar started_at_ = -1.0F;
    Status status_ = Status::InProgress;

    std::array<math::CumulativeAverageComputer<>, 3> averagers_;

    Modulator modulator_;

    math::SimpleMovingAverageFilter<500, Vector<2>> currents_filter_;
    math::SimpleMovingAverageFilter<500, Scalar> Ud_filter_;

    // State variables
    Scalar angular_velocity_ = 0;
    Scalar angular_position_ = 0;
    Scalar min_Iq_ = std::numeric_limits<Scalar>::infinity();
    Vector<2> Idq_ = Vector<2>::Zero();
    Vector<2> Udq_ = Vector<2>::Zero();
    Scalar phi_ = 0;

    // Rotor stall detection logic
    math::FivePointDifferentiator<Scalar> Iprime_estimator_;
    math::SampleVariance<Scalar> Iprime_stdev_estimator_;
    Scalar Iprime_ = 0;
    Scalar Iprime_mean_ = 0;
    Scalar Iprime_stdev_ = 0;

    variable_tracer::ProbeGroup<9> probes_;

public:
    FluxLinkageTask(SubTaskContextReference context,
                    const MotorParameters& initial_parameters) :
        context_(context),
        result_(initial_parameters),
        initial_Uq_((initial_parameters.max_current * context.params.motor_id.fraction_of_max_current) *
                    result_.rs * 1.5F),
        modulator_(result_.lq,
                   result_.rs,
                   result_.max_current,
                   context.params.controller.voltage_modulator_bandwidth,
                   context.board.pwm,
                   Modulator::DeadTimeCompensationPolicy::Disabled,
                   Modulator::CrossCouplingCompensationPolicy::Disabled),
        currents_filter_(Vector<2>::Zero()),
        Ud_filter_(0.0F),

        probes_("Ud",   &Udq_[0],
                "Uq",   &Udq_[1],
                "Id",   &Idq_[0],
                "Iq",   &Idq_[1],
                "AVel", &angular_velocity_,
                "phi",  &phi_,
                "miIp", &Iprime_,
                "miIm", &Iprime_mean_,
                "miIs", &Iprime_stdev_)
    {
        Udq_[1] = initial_Uq_;

        result_.phi = 0;

        if (!context_.params.motor_id.isValid() ||
            !result_.getRsLimits().contains(result_.rs) ||
            !result_.getLqLimits().contains(result_.lq) ||
            !os::float_eq::positive(result_.max_current))
        {
            status_ = Status::Failed;
        }
    }

    void onMainIRQ(Const, const board::motor::Status&) override { }

    void onNextPWMPeriod(const Vector<2>& phase_currents_ab, Const inverter_voltage) override
    {
        if (status_ != Status::InProgress)
        {
            return;
        }

        if (started_at_ < 0)
        {
            started_at_ = context_.getTime();
        }

        /*
         * Voltage modulation and filter update
         */
        Const low_pass_filter_innovation = context_.board.pwm.period * 10.0F;

        if (Udq_[1] > MinVoltage)
        {
            Modulator::Setpoint setpoint;
            setpoint.mode = Modulator::Setpoint::Mode::Uq;
            setpoint.value = Udq_[1];

            const auto out = modulator_.onNextPWMPeriod(phase_currents_ab,
                                                        inverter_voltage,
                                                        angular_velocity_,
                                                        angular_position_,
                                                        setpoint);
            context_.setPWM(out.pwm_setpoint);
            angular_position_ = out.extrapolated_angular_position;

            // Current filter update
            currents_filter_.update(out.Idq);
            Idq_ += low_pass_filter_innovation * (currents_filter_.getValue() - Idq_);

            // Ud filter update (Uq is known exactly since we use it as a setpoint)
            Ud_filter_.update(out.reference_Udq[0]);
            Udq_[0] += low_pass_filter_innovation * (Ud_filter_.getValue() - Udq_[0]);
        }
        else
        {
            // Voltage is not in the valid range, aborting
            result_.phi = 0;
            status_ = Status::Failed;
            IRQDebugOutputBuffer::setStringPointerFromIRQ("Voltage below the minimum");
            return;
        }

        Const I_norm = Idq_.norm();

        Iprime_ += low_pass_filter_innovation *
            (Iprime_estimator_.update(I_norm) / context_.board.pwm.period - Iprime_);

        /*
         * Phi computation (only if spinning fast enough to avoid division by zero)
         */
        if (angular_velocity_ > 1.0F)
        {
            Const new_phi = (Udq_.norm() - I_norm * result_.rs) / angular_velocity_;

            phi_ += context_.board.pwm.period * (new_phi - phi_);
        }

        /*
         * Acceleration/measurement
         */
        if (angular_velocity_ < context_.params.motor_id.phi_estimation_electrical_angular_velocity)
        {
            // Acceleration
            angular_velocity_ += (context_.params.motor_id.phi_estimation_electrical_angular_velocity /
                                  (VoltageSlopeLengthSec / 2.0F)) * context_.board.pwm.period;
        }
        else
        {
            if (Idq_[1] < min_Iq_)
            {
                min_Iq_ = Idq_[1];
                result_.phi = phi_;
            }

            if (result_.phi >= 0.0F)
            {
                // Stall detection
                if ((Iprime_ > 0.0F) &&
                    (Iprime_stdev_estimator_.getNumSamples() > MinSamplesBeforeStallDetectionEnabled) &&
                    ((Iprime_ - Iprime_mean_) > (Iprime_stdev_ * StallDetectionCurrentRateStdevMultiplier)))
                {
                    status_ = Status::Succeeded;
                }
                else
                {
                    // Minimum is not reached yet, continuing to reduce voltage
                    Udq_[1] -= (initial_Uq_ / VoltageSlopeLengthSec) * context_.board.pwm.period;

                    // Current change rate statistical analysis
                    // It is important to update the statistics AFTER the stall detection has been performed!
                    Iprime_stdev_estimator_.addSample(Iprime_);
                    if (Iprime_stdev_estimator_.areEstimatesAvailable())
                    {
                        Iprime_mean_ = Iprime_stdev_estimator_.getMean();
                        Iprime_stdev_ = Iprime_stdev_estimator_.getStandardDeviation();
                    }
                }
            }
            else
            {
                // Negative Phi value encountered at any point indicates that the Rs value is likely too high
                status_ = Status::Failed;
                IRQDebugOutputBuffer::setStringPointerFromIRQ("Negative Phi, resistance measurement is likely invalid");
            }
        }
    }

    Status getStatus() const override { return status_; }

    MotorParameters getEstimatedMotorParameters() const override { return result_; }
};

}
}
