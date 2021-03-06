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

#include <cstdint>
#include <array>
#include <zubax_chibios/os.hpp>
#include <hal.h>
#include <zubax_chibios/util/heapless.hpp>
#include <zubax_chibios/config/config.hpp>
#include <math/math.hpp>


namespace board
{
/**
 * This function should be invoked immediately from main().
 * It initializes a watchdog timer instance ASAP, which ensures that the application
 * will not stuck forever if initialization fails.
 */
os::watchdog::Timer init(unsigned watchdog_timeout_msec,
                         os::config::IStorageBackend& cfg_backend);

/**
 * Triggers an OS panic with the specified reason code printed into the serial console.
 */
__attribute__((noreturn))
void die(int reason);

/**
 * Resets the MCU via NVIC, no additional actions are performed.
 */
void restart();

/**
 * Returns the 128-bit hardware UID, where only the first 96 bit are used, and the rest is
 * filled with zeros.
 */
typedef std::array<std::uint8_t, 16> UniqueID;
UniqueID readUniqueID();

/**
 * RSA-1024 Signature of Authenticity management.
 * The signature can be written only once and it is typically done by the hardware manufacturer.
 * The read will fail if there is no signature installed.
 */
typedef std::array<std::uint8_t, 128> DeviceSignature;
bool tryReadDeviceSignature(DeviceSignature& out_sign);
bool tryWriteDeviceSignature(const DeviceSignature& sign);

/**
 * @ref detectHardwareVersion()
 */
struct HardwareVersion
{
    std::uint8_t major;
    std::uint8_t minor;

    auto toString() const
    {
        return os::heapless::format("%u.%u", major, minor);
    }
};

/**
 * The major code can be specified either by the hardware ID pins or at compile time.
 * The minor code is specified only by the hardware ID pins.
 */
HardwareVersion detectHardwareVersion();

/**
 * Sets the LED brightness and color. Brightness is specified per channel in the range [0, 1].
 */
using RGB = math::Vector<3>;
void setRGBLED(const RGB& rgb);

/**
 * Testpoint control.
 * The functions are made inline to ensure minimal runtime overhead even with LTO disabled.
 * @{
 */
inline void setTestPointA(bool level)
{
    palWritePad(GPIOD, GPIOD_TEST_1, level);
}

inline void setTestPointB(bool level)
{
    palWritePad(GPIOB, GPIOB_TEST_2, level);
}

inline void setTestPointC(bool level)
{
    palWritePad(GPIOB, GPIOB_TEST_3, level);
}

inline void setTestPointD(bool level)
{
    palWritePad(GPIOC, GPIOC_TEST_4, level);
}
/**
 * @}
 */

/**
 * Use this helper to toggle testpoints:
 *      RAIIToggler<setTestPointA> toggler;
 */
template <void Target(bool)>
struct RAIIToggler
{
    RAIIToggler()  { Target(true); }
    ~RAIIToggler() { Target(false); }
};

/**
 * Measures time intervals starting from the point it was created.
 * The class uses the DWT counter, so the maximum duration it can measure is very limited.
 */
class SmallTimeIntervalMeasurer
{
    const std::uint32_t started_at_ = DWT->CYCCNT;

    static float convertToSecond(std::uint32_t value) { return float(value) / float(STM32_SYSCLK); }

public:
    /**
     * Returns the number of seconds passed since the interval measurement started.
     */
    float sample() const
    {
        return convertToSecond(DWT->CYCCNT - started_at_);
    }
};

}
