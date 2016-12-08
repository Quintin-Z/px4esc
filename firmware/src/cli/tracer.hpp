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

#include <variable_tracer.hpp>
#include <zubax_chibios/os.hpp>
#include <cstring>
#include <unistd.h>


namespace cli
{
/**
 * This class runs an instance of variable tracer.
 * The encoded data gets reported into the standard output interface (either USB or UART).
 * Each output line is prefixed with '~', in order to facilitate its separation from regular CLI output.
 */
class Tracer : private chibios_rt::BaseStaticThread<512>
{
    using Impl = variable_tracer::Tracer<>;

    static constexpr unsigned WriteTimeoutMSec = 10;

    /// This character is not used by the Base85 encoding, so it's pretty unambiguous
    static constexpr char OutputPrefixCharacter = '~';

    chibios_rt::Mutex mutex_;
    Impl tracer_;
    char output_buffer_[Impl::MaxOutputStringLengthWithNullTermination + 3] = { OutputPrefixCharacter };


    void main() override
    {
        setName("tracer");

        auto last_loop_systime = chVTGetSystemTimeX();

        while (!os::isRebootRequested())
        {
            /*
             * Sleeping for minimum possible time interval in order to avoid buffer congestion and over-sampling.
             * Ideally, the duration of the pause should be about 100~200 microseconds, but the scheduler has to
             * round up the intervals to the tick period.
             */
            {
                const auto new_systime = chVTGetSystemTimeX();
                if (last_loop_systime == new_systime)
                {
                    chThdSleep(1);          // Enforcing about one sample per OS tick
                    last_loop_systime++;
                }
                else
                {
                    last_loop_systime = new_systime;
                }
            }

            std::pair<unsigned, Impl::SampleResult> result;

            {
                os::MutexLocker locker(mutex_);
                result = tracer_.sample(&output_buffer_[1]);
            }

            if (result.second == Impl::SampleResult::NothingToSample)
            {
                ::usleep(100000);
            }
            else
            {
                // Note that there's no real need to zero-terminate the output, since we treat the string as bytes
                output_buffer_[result.first + 1] = '\r';
                output_buffer_[result.first + 2] = '\n';

                os::MutexLocker stdio_locker(os::getStdIOMutex());

                (void) chnWriteTimeout(os::getStdIOStream(),
                                       reinterpret_cast<const std::uint8_t*>(&output_buffer_[0]),
                                       result.first + 3,
                                       MS2ST(WriteTimeoutMSec));
            }
        }
    }

public:
    using NameList = std::array<variable_tracer::ShortName, Impl::MaxTracedVariables>;

    struct RAIIEnabler
    {
        Tracer& target;

        RAIIEnabler(Tracer& target, const NameList& names) :
            target(target)
        {
            target.trace(names);
        }

        ~RAIIEnabler()
        {
            target.stop();
        }
    };

    virtual ~Tracer() { }

    void init(const ::tprio_t thread_priority)
    {
        os::MutexLocker locker(mutex_);
        (void) this->start(thread_priority);
    }

    void trace(const NameList& names)
    {
        os::MutexLocker locker(mutex_);
        tracer_.stop();
        for (auto n : names)
        {
            if (!n.isEmpty())
            {
                (void) tracer_.trace(n);
            }
        }
    }

    void stop()
    {
        os::MutexLocker locker(mutex_);
        tracer_.stop();
    }
};

}
