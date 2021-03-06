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

#include "transforms.hpp"
#include "observer/observer.hpp"
#include "motor_id/parameters.hpp"
#include <zubax_chibios/util/heapless.hpp>
#include <zubax_chibios/util/float_eq.hpp>
#include <math/math.hpp>
#include <cstdint>
#include <cassert>


namespace foc
{

using math::Scalar;
using math::Const;
using math::Vector;

/**
 * This structure entails all information about the connected load.
 * Some of these parameters can be automatically identified.
 * Values that are initialized to zero are considered undefined by default, and should be provided by the application.
 */
struct MotorParameters
{
    /**
     * Number of magnetic poles (not pairs!); must be a positive even number.
     * This is a mandatory parameter.
     */
    std::uint_fast8_t num_poles = 0;

    /**
     * Max phase current, Ampere.
     * This is a mandatory parameter.
     */
    Scalar max_current = 0;

    /**
     * Min phase current for stable observer operation. [ampere]
     * If not specified, this parameter will be automatically inferred from the max current;
     * @ref deduceMissingParameters().
     */
    Scalar min_current = 0;

    /**
     * Initial current used during spinup. [ampere]
     * If not specified, this parameter will be automatically inferred from the max current;
     * @ref deduceMissingParameters().
     */
    Scalar spinup_current = 0;

    /**
     * Magnetic field flux linkage. [weber]
     * This is a mandatory parameter; it can be estimated using the motor identification procedure.
     */
    Scalar phi = 0;

    /**
     * Phase resistance (half of phase-to-phase resistance) at the typical working temperature. [ohm]
     * This is a mandatory parameter; it can be estimated using the motor identification procedure.
     */
    Scalar rs = 0;

    /**
     * Phase inductance in the quadrature axis. [henry]
     * This is a mandatory parameter; it can be estimated using the motor identification procedure.
     */
    Scalar lq = 0;

    /**
     * Min electric angular velocity for stable operation. [radian/second]
     * Default value is provided, so this parameter is optional.
     */
    Scalar min_electrical_ang_vel = 200;

    /**
     * Current setpoint slope. [ampere/second]
     * Used in all control modes except voltage control.
     */
    Scalar current_ramp_amp_per_s = 300;

    /**
     * Voltage setpoint slope. [volt/second]
     * Used only in voltage control modes.
     */
    Scalar voltage_ramp_volt_per_s = 10;


    static math::Range<> getPhiLimits()
    {
        return {   0.02e-3F,
                 100.00e-3F };
    }

    static math::Range<> getRsLimits()
    {
        return { 0.002F,
                20.00F };
    }

    static math::Range<> getLqLimits()
    {
        return {    1e-6F,
                 2000e-6F };
    }


    void deduceMissingParameters()
    {
        if (!os::float_eq::positive(min_current) &&
            os::float_eq::positive(max_current))
        {
            min_current = max_current * 0.02F;
        }

        if (!os::float_eq::positive(spinup_current) &&
            os::float_eq::positive(max_current))
        {
            spinup_current = max_current * 0.85F;
        }
    }

    Scalar computeMinVoltage() const
    {
        Const min_rpmm = convertRotationRateElectricalToMechanical(convertAngularVelocityToRPM(min_electrical_ang_vel),
                                                                   num_poles);
        Const kv = convertFluxLinkageToKV(phi, num_poles);
        Const min_voltage_from_rpm = min_rpmm / kv;

        Const min_voltage_from_current = min_current * rs * 1.5F;

        Const min_voltage = std::max(min_voltage_from_rpm,
                                     min_voltage_from_current);

        if (math::Range<>(0.01F, 10.0F).contains(min_voltage))
        {
            return min_voltage;
        }
        else
        {
            return 0;
        }
    }

    bool isValid() const
    {
        static const auto is_positive = [](Const x) { return (x > 0) && std::isfinite(x); };

        return
            (num_poles >= 2)                         &&
            (num_poles % 2 == 0)                     &&
            is_positive(max_current)                 &&
            is_positive(min_current)                 &&
            is_positive(spinup_current)              &&
            (max_current > min_current)              &&
            (max_current >= spinup_current)          &&
            (spinup_current > min_current)           &&
            getPhiLimits().contains(phi)             &&
            getRsLimits().contains(rs)               &&
            getLqLimits().contains(lq)               &&
            is_positive(min_electrical_ang_vel)      &&
            is_positive(current_ramp_amp_per_s)      &&
            is_positive(voltage_ramp_volt_per_s);
    }

    auto toString() const
    {
        const bool num_poles_known = (num_poles >= 2) && (num_poles % 2 == 0);

        Scalar kv = 0;
        if ((phi > 0) && num_poles_known)
        {
            kv = convertFluxLinkageToKV(phi, num_poles);
        }

        Scalar min_rpmm = 0;
        if ((min_electrical_ang_vel > 0) && num_poles_known)
        {
            min_rpmm = convertRotationRateElectricalToMechanical(convertAngularVelocityToRPM(min_electrical_ang_vel),
                                                                 num_poles);
        }

        return os::heapless::String<240>(
            "Npols: %u\n"
            "Imax : %-7.2f A\n"
            "Imin : %-7.2f A\n"
            "Ispup: %-7.2f A\n"
            "Phi  : %-7.3f mWb, %.1f RPMM/V\n"
            "Rs   : %-7.3f Ohm\n"
            "Lq   : %-7.2f uH\n"
            "Wmin : %-7.1f rad/s, %.1f RPMM\n"
            "Iramp: %-7.1f A/s\n"
            "Vramp: %-7.1f V/s\n"
            "Valid: %s").format(
            unsigned(num_poles),
            double(max_current),
            double(min_current),
            double(spinup_current),
            double(phi) * 1e3, double(kv),
            double(rs),
            double(lq) * 1e6,
            double(min_electrical_ang_vel), double(min_rpmm),
            double(current_ramp_amp_per_s),
            double(voltage_ramp_volt_per_s),
            isValid() ? "YES" : "NO");
    }
};

struct ControllerParameters
{
    /// Preferred duration of spinup, real duration may slightly differ, seconds
    Scalar nominal_spinup_duration = 1.5F;

    /// If the rotor stalled this many times in a row, latch into FAULT state
    std::uint32_t num_stalls_to_latch = 100;

    /// Multiplier of the P term of the current PI controllers
    Scalar voltage_modulator_bandwidth = 0.05F;

    /// Use additional correction of the PID DQ voltage outputs; refer to the theory for explanation
    bool voltage_modulator_cross_coupling_inductance_compensation = false;

    bool isValid() const
    {
        return math::Range<>(0.1F, 60.0F).contains(nominal_spinup_duration) &&
               os::float_eq::positive(voltage_modulator_bandwidth)          &&
               num_stalls_to_latch > 0;
    }

    auto toString() const
    {
        return os::heapless::format("Tspinup: %.1f sec\n"
                                    "Nslatch: %u\n"
                                    "VM Bndw: %.3f\n"
                                    "VM CCIC: %s",
                                    double(nominal_spinup_duration),
                                    unsigned(num_stalls_to_latch),
                                    double(voltage_modulator_bandwidth),
                                    voltage_modulator_cross_coupling_inductance_compensation ? "ON" : "OFF");
    }
};

/**
 * Constant parameters shared between tasks.
 * This data is guaranteed to stay constant as long as a task is running,
 * but it may be changed when tasks are switched (e.g. configuration parameters may be updated at run time).
 */
struct Parameters
{
    ControllerParameters controller;
    MotorParameters motor;
    motor_id::Parameters motor_id;
    observer::Parameters observer;


    bool isValid() const
    {
        return controller.isValid() &&
               motor.isValid()      &&
               motor_id.isValid()   &&
               observer.isValid();
    }

    auto toString() const
    {
        static const auto append = [](auto& s, const char* name, const auto& src)
        {
            s.concatenate(name, ":\n", src.toString(), "\n--\n");
        };

        os::heapless::String<400> s;

        append(s, "Controller", controller);
        append(s, "Motor",      motor);
        append(s, "Motor ID",   motor_id);
        append(s, "Observer",   observer);

        s.concatenate("Valid: ", isValid() ? "YES" : "NO");

        return s;
    }
};

}
