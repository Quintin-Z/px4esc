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
#include <zubax_chibios/util/helpers.hpp>


namespace foc
{
namespace motor_id
{
namespace test_run
{
/**
 * An interface that abstracts the test run pattern generation logic.
 */
class IPatternGenerator
{
public:
    struct Output
    {
        Scalar relative_setpoint = 0;               ///< Unitless, [-1, 1]

        Scalar electrical_angular_velocity = 0;     ///< [electrical radian/second]

        /**
         * Instructs the outer logic to stop the rotor.
         * This may introduce an unspecified delay until the next invocation of @ref update().
         */
        bool spin_down = false;

        /**
         * True if the output contains step change of any parameter.
         * This will hint the stall detector about an upcoming disturbance.
         */
        bool step_change = false;
    };

    virtual ~IPatternGenerator() { }

    /**
     * Invoked periodically in order to compute the next step of the pattern.
     * @param period                    invocation period, must remain constant [second]
     * @param prev_angular_velocity     last angular velocity output
     * @param prev_relative_setpoint    last relative setpoint output
     * @return                          next step of the pattern
     */
    virtual Output update(Const period, Const prev_angular_velocity, Const prev_relative_setpoint) = 0;

    /**
     * Invoked when the outer logic detects that the rotor has stalled.
     * This puts the outer logic and the motor into the same state that could be achieved by returning @ref Output
     * with spin_down = true.
     */
    virtual void onStallDetected() = 0;

    /**
     * @return true if the current pattern is finished.
     */
    virtual bool isFinished() const = 0;
};

/**
 * Simple linear pattern, zero to max.
 */
class LinearPatternGenerator : public IPatternGenerator
{
    const MotorParameters motor_;

    Const max_angacc_ = 2000.0F;    // TODO: Make this configurable!
    Const max_angvel_ = 5000.0F;    // TODO: Make this configurable!

    unsigned iteration_ = 0;
    bool finished_ = false;

    template <class T, unsigned N>
    static constexpr unsigned getArraySize(const T (&)[N]) noexcept { return N; }

public:
    LinearPatternGenerator(const MotorParameters& motor) :
        motor_(motor)
    { }

    Output update(Const period, Const prev_angular_velocity, Const prev_relative_setpoint) override
    {
        static constexpr Scalar MaxAngAccMults[] =
        {
            0.01F,
            0.02F,
            0.10F,
            0.20F,
            0.50F,
            1.00F
        };

        static const std::function<Scalar (Const)> SetpointTransferFuncs[] =
        {
            // Constant setpoint
            [](Const) { return 0.10F; },
            [](Const) { return 0.25F; },
            [](Const) { return 0.40F; },
            [](Const) { return 0.55F; },
            [](Const) { return 0.70F; },
            [](Const) { return 0.85F; },
            [](Const) { return 1.00F; },
            // Positive rate setpoint
            [](Const x) { return 0.1F + 0.1F * x; },
            [](Const x) { return 0.1F + 0.2F * x; },
            [](Const x) { return 0.1F + 0.5F * x; },
            [](Const x) { return 0.1F + 1.0F * x; },
            [](Const x) { return 0.1F + 2.0F * x; },
            [](Const x) { return 0.1F + 5.0F * x; },
            [](Const x) { return 0.3F + 0.1F * x; },
            [](Const x) { return 0.3F + 0.5F * x; },
            [](Const x) { return 0.3F + 1.0F * x; },
            // TODO: add squares?
            // Negative rate setpoint
            [](Const x) { return 0.5F - 0.1F * x; },
            [](Const x) { return 0.6F - 0.2F * x; },
            [](Const x) { return 0.7F - 0.5F * x; },
            [](Const x) { return 1.0F - x; },
            [](Const x) { return 2.0F - x; }
        };

        constexpr unsigned NumIterations = getArraySize(MaxAngAccMults) * getArraySize(SetpointTransferFuncs);
        static_assert(NumIterations > 100, "Woah");
        if (iteration_ >= NumIterations)
        {
            finished_ = true;
            return Output();
        }

        const bool spinup_done = std::abs(prev_angular_velocity) > motor_.min_electrical_ang_vel;

        /*
         * Computing new angular velocity as a function of time
         */
        Scalar angular_acceleration = 0;

        if (spinup_done)
        {
            angular_acceleration = max_angacc_ * MaxAngAccMults[iteration_ % getArraySize(MaxAngAccMults)];
        }
        else
        {
            angular_acceleration = max_angacc_ * MaxAngAccMults[0];     // Constraining acceleration
        }

        Const abs_angvel = std::abs(prev_angular_velocity) + period * angular_acceleration;

        Const norm_angvel = abs_angvel / max_angvel_;

        /*
         * Computing new setpoint as a function of angular velocity normalized in [0%, 100%]
         */
        Scalar raw_setpoint = SetpointTransferFuncs[iteration_ / getArraySize(MaxAngAccMults)](norm_angvel);

        if (!spinup_done)
        {
            Const step = period * motor_.current_ramp_amp_per_s * 0.1F;
            Const abs_prev_relative_setpoint = std::abs(prev_relative_setpoint);

            if (raw_setpoint >= abs_prev_relative_setpoint)         // Avoiding current spikes during spinup
            {
                raw_setpoint = abs_prev_relative_setpoint + step;
            }
            else
            {
                raw_setpoint = abs_prev_relative_setpoint - step;
            }
        }

        /*
         * Output, reversing as necessary
         */
        if ((abs_angvel >= max_angvel_) || (raw_setpoint < 0.0F))
        {
            iteration_++;
            Output output;
            output.spin_down = true;
            return output;
        }

        Const clipped_setpoint = math::Range<>(0.0F, 1.0F).constrain(raw_setpoint);

        const bool reverse = (iteration_ & 1) == 1;
        Output output;
        if (reverse)
        {
            output.electrical_angular_velocity = -abs_angvel;
            output.relative_setpoint           = -clipped_setpoint;
        }
        else
        {
            output.electrical_angular_velocity = abs_angvel;
            output.relative_setpoint           = clipped_setpoint;
        }

        return output;
    }

    void onStallDetected() override { iteration_++; }

    bool isFinished() const override { return finished_; }
};

/**
 * A container class that holds and switches pattern generators.
 */
template <typename... PatternList>
class PatternCollection
{
    PatternCollection(const volatile PatternCollection&) = delete;
    PatternCollection(const volatile PatternCollection&&) = delete;
    PatternCollection& operator=(const volatile PatternCollection&) = delete;
    PatternCollection& operator=(const volatile PatternCollection&&) = delete;

    typedef TypeEnumeration<PatternList...> Patterns;
    friend Patterns;   // This is needed for type resolution methods

    const MotorParameters motor_params_;
    IPatternGenerator* current_pattern_ = nullptr;

    alignas(Patterns::LargestAlignment) mutable std::uint8_t pool_[Patterns::LargestSize];

    template <typename T>
    IPatternGenerator* onTypeResolutionSuccess() const
    {
        static_assert(sizeof(T) <= sizeof(pool_), "Congratulations, you broke your C++ compiler!");
        return new (pool_) T(motor_params_);
    }

    IPatternGenerator* onTypeResolutionFailure() const { return nullptr; }

public:
    PatternCollection(const MotorParameters& motor_params) :
        motor_params_(motor_params)
    { }

    ~PatternCollection() { destroyCurrentPattern(); }

    static constexpr unsigned getNumPatterns() { return Patterns::Length; }

    /**
     * Destroys the current pattern, resets the current pattern pointer to nullptr.
     */
    void destroyCurrentPattern()
    {
        if (current_pattern_ != nullptr)
        {
            current_pattern_->~IPatternGenerator();
            current_pattern_ = nullptr;
        }
    }

    /**
     * Selects the new pattern specified by index; if the index is out of bounds, selects no pattern and resets the
     * current pattern pointer to nullptr.
     */
    void selectPatternByIndex(unsigned index)
    {
        destroyCurrentPattern();
        current_pattern_ = Patterns::findTypeByID(*this, index);
    }

    IPatternGenerator* getCurrentPattern() { return current_pattern_; }
};

/**
 * Simple rotor stall detection logic
 */
class StallDetector
{
    static constexpr unsigned NumInitSamples = 10000;

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
    StallDetector(const MotorParameters& motor_params) :
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

        Const lowpass_innovation =
            (vp2_variance_estimator_.getNumSamples() > (NumInitSamples / 2)) ?
                (period * 10.0F) : period;

        // Ang vel update
        // TODO Ask Dmitry to document the theory behind this equation
        Const Iq_prime = Iq_gradient_estimator_.update(Idq[1]) / period;
        Const new_output = std::abs((Udq[1] - m_.rs * Idq[1] - m_.lq * Iq_prime) / (m_.lq * Idq[0] + m_.phi));
        v_ += lowpass_innovation * (new_output - v_);

        // Ang vel derivatives update
        vp_ += lowpass_innovation * (vp_estimator_.update(v_) / period - vp_);

        vp2_ += lowpass_innovation * (vp2_estimator_.update(vp_) / period - vp2_);

        // Stall detection BEFORE statistics is updated
        if (vp2_variance_estimator_.getNumSamples() > NumInitSamples)
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
class Task : public ISubTask
{
    static constexpr Scalar MaxElectricalAngularVelocity    = 5000.0F;     ///< [rad/sec]
    static constexpr Scalar DelayBetweenPatterns            = 10.0F;
    static constexpr Scalar DelayAfterSpinDown              = 5.0F;

    /**
     * Note that we're using default settings of the three phase modulator.
     * It is important to use same settings that are used in normal operating mode,
     * otherwise model identification/adjustment may be rendered invalid.
     */
    using Modulator = ThreePhaseVoltageModulator<>;

    using Patterns = PatternCollection<LinearPatternGenerator>;

    SubTaskContextReference context_;
    const MotorParameters result_;

    Const min_stable_angular_velocity_;
    Const spin_down_abs_angular_acceleration_ = 400.0F;     // TODO: make configurable!

    Scalar remaining_delay_ = 0;

    Scalar angular_velocity_ = 0;

    os::helpers::LazyConstructor<Modulator> modulator_;
    Modulator::Setpoint setpoint_;
    Modulator::Output latest_modulator_output_;

    Patterns pattern_collection_;
    unsigned pattern_index_ = 0;

    bool spin_down_ = false;

    StallDetector stall_detector_;

    Status status_ = Status::InProgress;

    const variable_tracer::ProbeGroup<5> probes_;


    void resetModulator()
    {
        AbsoluteCriticalSectionLocker locker;

        modulator_.construct(result_.lq,
                             result_.rs,
                             result_.max_current,
                             context_.params.controller.voltage_modulator_bandwidth,
                             context_.board.pwm,
                             Modulator::DeadTimeCompensationPolicy::Disabled,
                             Modulator::CrossCouplingCompensationPolicy::Disabled);

        latest_modulator_output_ = Modulator::Output();
    }

public:
    // trace -c time AVel Iq sdSD
    Task(SubTaskContextReference context, const MotorParameters& result) :
        context_(context),
        result_(result),
        min_stable_angular_velocity_(result.min_electrical_ang_vel * 2.0F),

        setpoint_{0.0F, Modulator::Setpoint::Mode::Iq},

        pattern_collection_(result),

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

        resetModulator();
    }

    void onMainIRQ(Const period, const board::motor::Status&) override
    {
        if (status_ != Status::InProgress)
        {
            return;
        }

        if (spin_down_)
        {
            Const step = period * spin_down_abs_angular_acceleration_;

            if (std::abs(angular_velocity_) <= (2.0F * step))
            {
                angular_velocity_ = 0;
                setpoint_.value = 0;
                spin_down_ = false;
                stall_detector_.reset();
                resetModulator();
                remaining_delay_ = DelayAfterSpinDown;

                IRQDebugOutputBuffer::setStringPointerFromIRQ("Spindown completed");
            }
            else
            {
                if (angular_velocity_ > 0)
                {
                    angular_velocity_ -= step;
                }
                else
                {
                    angular_velocity_ += step;
                }
            }

            return;
        }

        if (remaining_delay_ > 0)
        {
            setpoint_.value = 0;
            angular_velocity_ = 0;
            stall_detector_.reset();
            remaining_delay_ -= period;
            return;
        }

        /*
         * Pattern selection
         */
        if (pattern_collection_.getCurrentPattern() == nullptr)
        {
            IRQDebugOutputBuffer::setStringPointerFromIRQ("Next pattern...");
            pattern_collection_.selectPatternByIndex(pattern_index_);
            stall_detector_.reset();
            resetModulator();
        }

        IPatternGenerator* const pattern = pattern_collection_.getCurrentPattern();
        if (pattern == nullptr)
        {
            status_ = Status::Succeeded;
            return;
        }

        /*
         * Stall detection update
         */
        {
            Vector<2> Idq;
            Vector<2> Udq;

            {
                AbsoluteCriticalSectionLocker locker;
                Idq = latest_modulator_output_.Idq;
                Udq = latest_modulator_output_.reference_Udq;
            }

            stall_detector_.update(period, Idq, Udq);
        }

        /*
         * Pattern update
         */
        if (stall_detector_.isStallDetected())
        {
            pattern->onStallDetected();
            angular_velocity_ = 0;
            setpoint_.value = 0;
            spin_down_ = true;
            return;
        }

        if (pattern->isFinished())
        {
            spin_down_ = true;
            pattern_collection_.destroyCurrentPattern();
            pattern_index_++;
            remaining_delay_ = DelayBetweenPatterns;
            return;
        }

        const auto output = pattern->update(period, angular_velocity_, setpoint_.value / result_.max_current);

        if (!output.spin_down)
        {
            angular_velocity_ = output.electrical_angular_velocity;
            setpoint_.value = output.relative_setpoint * result_.max_current;

            if (output.step_change)
            {
                stall_detector_.expectFalsePositiveAhead();
            }
        }
        else
        {
            // Decelerating the rotor keeping the same setpoint
            spin_down_ = true;
            setpoint_.value = std::copysign(setpoint_.value, angular_velocity_);
        }
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
            modulator_->onNextPWMPeriod(phase_currents_ab,
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
}
