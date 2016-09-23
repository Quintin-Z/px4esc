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

#include "esc_controller.hpp"
#include <uavcan/equipment/esc/RPMCommand.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <zubax_chibios/os.hpp>
#include <foc/foc.hpp>
#include <cstdint>


namespace uavcan_node
{
namespace esc_controller
{
namespace
{

const auto StatusTransferPriority = uavcan::TransferPriority::fromPercent<75>();


os::config::Param<std::uint8_t> g_param_esc_index                  ("uavcan.esc_indx",     0,      0,      15);
os::config::Param<float>        g_param_esc_cmd_ttl                ("uavcan.esc_ttl",   0.3F,   0.1F,   10.0F);
os::config::Param<float>        g_param_esc_status_interval        ("uavcan.esc_si",   0.05F,  0.01F,    1.0F);
os::config::Param<float>        g_param_esc_status_interval_passive("uavcan.esc_sip",   0.5F,  0.01F,   10.0F);


uavcan::LazyConstructor<uavcan::Publisher<uavcan::equipment::esc::Status>> g_pub_status;
uavcan::LazyConstructor<uavcan::Timer> g_timer;

std::uint8_t g_self_index;
float g_command_ttl;


void cbRawCommand(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand>& msg)
{
    if (msg.cmd.size() > g_self_index)
    {
        const float command =
            float(msg.cmd[g_self_index]) /
            float(uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max());

        foc::setSetpoint(foc::ControlMode::RatiometricCurrent, command, g_command_ttl);
    }
}


void cbRPMCommand(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RPMCommand>& msg)
{
    (void) msg;
    // TODO: Closed loop operation
}


void cbTimer(const uavcan::TimerEvent& event)
{
    const auto state = foc::getState();

    /*
     * Scheduling the next timer event depending on the current state
     */
    {
        static const float interval_normal  = g_param_esc_status_interval.get();
        static const float interval_passive = g_param_esc_status_interval_passive.get();

        const bool is_passive = state == foc::State::Idle ||
                                state == foc::State::Fault;

        const auto current_interval = uavcan::MonotonicDuration::fromUSec(
            std::uint64_t((is_passive ? interval_passive : interval_normal) * 1e6F));

        g_timer->startOneShotWithDeadline(event.scheduled_time + current_interval);
    }

    /*
     * Publishing status
     */
    uavcan::equipment::esc::Status status;

    status.esc_index = g_self_index;

    status.voltage     = board::motor::getInverterVoltage();
    status.temperature = board::motor::getInverterTemperature();

    status.current = foc::getInstantCurrent();

    status.rpm = 0;     // TODO

    const auto max_power = foc::getMaximumPower();
    const auto instant_power = foc::getInstantPower();

    if (max_power > 0 &&
        instant_power > 0)
    {
        status.power_rating_pct = std::uint8_t(instant_power / max_power + 0.5F);
    }

    (void) g_pub_status->broadcast(status);
}

} // namespace

int init(uavcan::INode& node)
{
    static bool initialized = false;
    ASSERT_ALWAYS(!initialized);
    initialized = true;

    /*
     * Configuration parameters
     */
    g_self_index  = g_param_esc_index.get();
    g_command_ttl = g_param_esc_cmd_ttl.get();

    /*
     * Publishers
     */
    g_pub_status.construct<uavcan::INode&>(node);
    const int pub_init_res = g_pub_status->init(StatusTransferPriority);
    if (pub_init_res < 0)
    {
        return pub_init_res;
    }

    /*
     * Subscribers
     */
    {
        static uavcan::Subscriber<uavcan::equipment::esc::RawCommand> sub_raw_command(node);

        const int res = sub_raw_command.start(&cbRawCommand);
        if (res < 0)
        {
            return res;
        }
    }

    {
        static uavcan::Subscriber<uavcan::equipment::esc::RPMCommand> sub_rpm_command(node);

        const int res = sub_rpm_command.start(&cbRPMCommand);
        if (res < 0)
        {
            return res;
        }
    }

    /*
     * Timer
     */
    g_timer.construct<uavcan::INode&>(node);
    g_timer->setCallback(&cbTimer);
    // Arbitrary delay to kickstart the process
    g_timer->startOneShotWithDelay(uavcan::MonotonicDuration::fromMSec(1000));

    return 0;
}

}
}