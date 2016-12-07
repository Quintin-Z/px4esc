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
#include <cassert>
#include <cstring>
#include <type_traits>
#include <zubax_chibios/util/heapless.hpp>
#include <board/motor.hpp>

/**
 * This is a simple one-header library that enables simple real-time tracing of arbitrary variables of the application.
 */
namespace variable_tracer
{
/**
 * Critical section must be defined according to the application's requirements.
 */
using CriticalSectionLocker = board::motor::AbsoluteCriticalSectionLocker;

/**
 * Short variable name, allows for very fast name matching (virtually 1 cycle).
 * Maximum length is limited and depends on the platform:
 *  - 2 characters on 16-bit platforms
 *  - 4 characters on 32-bit platforms
 *  - 8 characters on 64-bit platforms
 */
class ShortName
{
public:
    typedef unsigned UnderlyingType;

    static constexpr unsigned MaxLength = sizeof(UnderlyingType);

private:
    UnderlyingType value_ = 0;

    void assign(const char* name)
    {
        value_ = 0;
        for (unsigned i = 0; (i < MaxLength) && (name[i] != '\0'); i++)
        {
            value_ |= UnderlyingType(name[i]) << (8 * i);
        }
    }

public:
    ShortName() { }

    ShortName(const char* name)     // Implicit
    {
        assign(name);
    }

    ShortName& operator=(const char* name)     // Implicit
    {
        assign(name);
        return *this;
    }

    bool operator==(const ShortName& rhs) const
    {
        return value_ == rhs.value_;
    }

    bool isEmpty() const { return value_ == 0; }

    os::heapless::String<MaxLength> toString() const
    {
        os::heapless::String<MaxLength> ret;

        for (unsigned i = 0; i < MaxLength; i++)
        {
            const char c = char(value_ >> (8 * i));
            if (c == '\0')
            {
                break;
            }
            ret.append(c);
        }

        return ret;
    }
};


class Probe;

/**
 * Do not use this class directly, it is for internal use only.
 */
class ProbeList final
{
    const Probe* head_ = nullptr;  // We use doubly linked list because we need constant-time insertion and removal

    std::uint_fast32_t modification_count_ = 0;

public:
    using ModificationCountType = decltype(modification_count_);

    static ProbeList& getInstance()
    {
        static ProbeList list;
        return list;
    }

    std::uint_fast32_t getModificationCount() const { return modification_count_; }

    void add(const Probe* const node);

    void remove(const Probe* const node);

    const Probe* getHead() const { return head_; }
};

/**
 * Use this class to declare a traceable named variable.
 * Instances of this class can be created and destroyed from any context, including IRQ -
 * all global state is protected with critical sections.
 */
class Probe final
{
    friend class ProbeList;

    Probe(Probe&) = delete;
    Probe(Probe&&) = delete;
    void operator=(Probe&) = delete;
    void operator=(Probe&&) = delete;

    mutable const Probe* next_ = nullptr;
    mutable const Probe* prev_ = nullptr;

    const ShortName name_;
    const volatile void* const location_ = nullptr;
    const std::uint_fast8_t size_ = 0;

public:
    /**
     * Maximum sizeof() of traced variable. Should be the size of the largest arithmetic type.
     */
    static constexpr std::uint_fast8_t MaxDataSize = std::max(sizeof(long long), sizeof(long double));

    /**
     * Manual constructor. It is not recommended to use it; refer to the templated overload instead.
     */
    Probe(const ShortName name,
          const volatile void* const location,
          const std::uint_fast8_t size) :
        name_(name),
        location_(location),
        size_(size)
    {
        assert(size > 0 && size <= MaxDataSize);
        ProbeList::getInstance().add(this);
    }

    /**
     * Use this constructor to build instances of this class.
     * Note that only arithmetic and enum types are accepted!
     * @tparam T        deduced automatically
     * @param name      name of the variable, 1 to 4 alphanumeric ASCII characters
     * @param location  typed address of the variable; the type is used for size deduction
     */
    template <typename T,
              typename = std::enable_if_t<std::is_arithmetic<T>::value || std::is_enum<T>::value>>
    Probe(const char* const name,
          const volatile T* const location) :
        Probe(name, location, sizeof(T))
    { }

    ~Probe()
    {
        ProbeList::getInstance().remove(this);
    }

    /**
     * @return ASCII human-readable name of the variable; use toString() to get string
     */
    ShortName getName() const { return name_; }

    /**
     * @return address of the variable in the local memory space
     */
    const volatile void* getLocation() const { return location_; }

    /**
     * @return size of the variable in bytes
     */
    std::uint_fast8_t getSize() const { return size_; }

    bool isSameName(const ShortName other) const { return name_ == other; }

    const Probe* getNext() const { return next_; }
};


inline void ProbeList::add(const Probe* const node)
{
    CriticalSectionLocker locker;
    (void) locker;

    modification_count_++;

    if (head_ == nullptr)
    {
        node->prev_ = nullptr;
        node->next_ = nullptr;
    }
    else
    {
        head_->prev_ = node;
        node->next_ = head_;
    }

    head_ = node;
}

inline void ProbeList::remove(const Probe* const node)
{
    CriticalSectionLocker locker;
    (void) locker;

    modification_count_++;

    if (node->prev_ == nullptr)
    {
        head_ = node->next_;
    }
    else
    {
        node->prev_->next_ = node->next_;
    }

    if (node->next_ != nullptr)
    {
        node->next_->prev_ = node->prev_;
    }
}

/**
 * A simple binary-to-text encoder algorithm.
 * This algorithm is based on Z85, which in turn is based on Base85. The difference from Z85 is that the byte order
 * is not enforced - a little-endian system will encode data in little-endian format, and a big-endian system will
 * encode data in the same way as the standard Z85 algorithm would.
 * This is like Base64 but much faster and more space efficient.
 * More on Z85 here: https://rfc.zeromq.org/spec:32/Z85/ and https://en.wikipedia.org/wiki/Ascii85
 */
class BinaryToTextEncoder
{
public:
    /**
     * Data to be encoded MUST be aligned at this boundary.
     */
    static constexpr unsigned DataBufferAlignment = 4;

    /**
     * Computes how much space is needed to encode a byte sequence of specified length into ASCII string,
     * plus null termination byte.
     * @param data_length   number of bytes to encode
     * @return              number of ASCII characters needed
     */
    static constexpr std::size_t predictEncodedStringLengthWithNullTermination(const std::size_t data_length)
    {
        return (((data_length + 3UL) & ~3UL) * 5UL / 4UL) + 1;
    }

    /**
     * Encodes data into ASCII string. Note that the input buffer must be aligned!
     * @param input         input data, MUST BE ALIGNED AT 4 BYTES
     * @param input_size    length of the input byte sequence; ROUNDED UP TO A MULTIPLE OF 4
     * @param output        pointer to the output ASCII buffer; @ref predictEncodedStringLengthWithNullTermination()
     * @return              length of the generated output string, not including the null termination byte
     */
    static unsigned encode(const void* const input,
                           const std::size_t input_size,
                           char* const output)
    {
        static constexpr std::uint32_t Base = 85;

        // We place the table into the data section in order to ensure that it ends up in RAM, for performance reasons
        static constexpr char Table[Base + 1]
#if __GNUC__
            __attribute__((section(".data")))
#endif
            = { "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ.-:+=^!/*?&<>()[]{}@%$#" };

        assert(std::size_t(input) % DataBufferAlignment == 0);

        const std::uint32_t* ip = static_cast<const std::uint32_t*>(input);     // PROPER ALIGNMENT IS REQUIRED
        const std::uint32_t* const input_end = ip + (input_size + 3U) / 4U;     // ROUNDING UP TO 4 BYTES
        char* op = output;

        while (ip < input_end)
        {
            const std::uint32_t value = *ip++;
            *op++ = Table[value / (Base * Base * Base * Base) % Base];
            *op++ = Table[value / (Base * Base * Base) % Base];
            *op++ = Table[value / (Base * Base) % Base];
            *op++ = Table[value / Base % Base];
            *op++ = Table[value % Base];
        }

        const unsigned output_length = unsigned(op - output);

        *op = '\0';

        return output_length;
    }
};

/**
 * Main variable tracer class.
 * Samples data and generates ASCII-encoded dumps in real time.
 *
 * Although the Probe objects can be created and destroyed from any context, the API of this class is not even
 * thread-safe, so proper thread synchronization should be taken care of, if applicable.
 *
 * The class can generate the following types of ASCII output strings:
 *
 *  - base85 encoded sample of the values of the traced variables; @ref BinaryToTextEncoder
 *
 *  - list of variables separated by the tab character; each variable entry contains its name and size separated by
 *    the right slash. This output string can be distinguished from the base85 encoded sample by the tab character
 *    being placed at the first position.
 *
 * Example output, in which the class is tracing one float64 and four float32 variables
 * (tabs were replaced with spaces):
 *
 *      A/8     B/4     C/4     D/4     E/4
 *  Nv>w%k}A+k0000000000%9$7H%9$7H
 *  gDl0Ek$xi90000000000%9$7HE$dI0
 *  +*WNik%H-70000000000%9$7H%9$7H
 *  /}e%zk$wjo0000000000%9$7HE$dI0
 *  br5yLk#CQ#0000000000%9$7H%9$7H
 *  g$89sl0yjj0000000000%9$7HE$dI0
 *  >BKWHl1x)$0000000000%9$7H%9$7H
 *
 * Note that the output is always padded at the end with garbage to 4 byte boundary. Keep that in mind when decoding.
 *
 * @tparam MaxTracedVariables_       maximum number of variables that can be traced concurrently
 */
template <unsigned MaxTracedVariables_ = 10>
class Tracer
{
public:
    static constexpr unsigned MaxTracedVariables = MaxTracedVariables_;

    static constexpr char VariableListSeparator = '\t';
    static constexpr char VariableNameSizeSeparator = '/';

private:
    struct CacheEntry
    {
        ShortName name;
        const volatile void* location = nullptr;
        std::uint_fast8_t size = 0;

        bool isEmpty() const { return location == nullptr; }
    };

    std::array<ShortName, MaxTracedVariables> traced_names_;
    std::array<CacheEntry, MaxTracedVariables> cache_;
    ProbeList::ModificationCountType var_list_mod_cnt_ = 0;


    bool isCacheStillValid() const
    {
        return ProbeList::getInstance().getModificationCount() == var_list_mod_cnt_;
    }

    void markCacheValid()
    {
        var_list_mod_cnt_ = ProbeList::getInstance().getModificationCount();
    }

    void markCacheInvalid()
    {
        var_list_mod_cnt_--;
    }

    static std::pair<const volatile void*, std::uint_fast8_t> getVariableLocationAndSizeByName(const ShortName name)
    {
        CriticalSectionLocker locker;                   // Remember that the list can be mutated concurrently
        auto w = ProbeList::getInstance().getHead();
        while (w != nullptr)
        {
            if (w->isSameName(name))
            {
                return {w->getLocation(), w->getSize()};
            }
            w = w->getNext();
        }
        return {nullptr, 0};
    }

    void rebuildCache()
    {
        std::fill(cache_.begin(), cache_.end(), CacheEntry());

        markCacheValid();       // Cache may be invalidated in the process, which is fine - we'll try again next time

        auto out = cache_.begin();
        for (auto n : traced_names_)
        {
            if (n.isEmpty())
            {
                break;
            }

            const auto loc_size = getVariableLocationAndSizeByName(n);
            if (loc_size.first != nullptr && loc_size.second > 0)
            {
                *out++ = {n, loc_size.first, loc_size.second};
            }
        }
    }

#if __GNUC__
    __attribute__((optimize("unroll-loops")))
#endif
    unsigned gatherDataIfCacheStillValid(std::uint8_t* const data_write_ptr) const
    {
        unsigned offset = 0;

        {
            // TODO: Profile this
            CriticalSectionLocker locker;
            if (isCacheStillValid())
            {
                for (auto& x : cache_)      // On small embedded targets, unrolling this should make it faster
                {
                    if (x.isEmpty())
                    {
                        break;
                    }
                    (void) std::memcpy(data_write_ptr + offset, const_cast<const void*>(x.location), x.size);
                    offset += x.size;
                }
            }
        }

        return offset;
    }

    unsigned generateVariableList(char* const output_ptr) const
    {
        char* op = output_ptr;

        for (auto& x : cache_)
        {
            if (x.isEmpty())
            {
                break;
            }

            *op++ = VariableListSeparator;

            {
                const auto str = x.name.toString();
                std::strncpy(op, str.c_str(), ShortName::MaxLength);
                op += str.length();
            }

            *op++ = VariableNameSizeSeparator;

            {
                const auto size = os::heapless::intToString<36>(x.size);
                std::strncpy(op, size.c_str(), ShortName::MaxLength);
                op += size.length();
            }
        }

        const unsigned len = unsigned(op - output_ptr);
        *op = '\0';
        return len;
    }

public:
    /**
     * Maximum possible length of ASCII output, including the null termination byte.
     */
    static constexpr std::size_t MaxOutputStringLengthWithNullTermination =
        std::max<std::size_t>(BinaryToTextEncoder::predictEncodedStringLengthWithNullTermination(
                                  MaxTracedVariables * Probe::MaxDataSize),
                              MaxTracedVariables * (ShortName::MaxLength + 4) + 1);

    /**
     * @ref sample().
     */
    enum SampleResult
    {
        NothingToSample,        ///< No data generated, nothing to do
        VariableListUpdated,    ///< Variable list may have been changed, e.g. new variables were added and such
        SampledSuccessfully     ///< Regular sample has been performed, ASCII dump has been generated
    };

    /**
     * Perform one sample and write the ASCII encoded dump into the provided output pointer.
     * The dump can then be sent to the outside world, e.g. via UART.
     * @param output_ptr    location where the ASCII output will be stored; must be not smaller than
     *                      @ref MaxOutputStringLengthWithNullTermination
     * @return              length of the encoded string and @ref SampleResult
     */
    std::pair<unsigned, SampleResult> sample(char* output_ptr)
    {
        if (traced_names_.front().isEmpty())
        {
            *output_ptr = '\0';
            return {0, SampleResult::NothingToSample};
        }

        {
            alignas(BinaryToTextEncoder::DataBufferAlignment)
            std::array<std::uint8_t, MaxTracedVariables * Probe::MaxDataSize> data_buffer;

            if (const unsigned size = gatherDataIfCacheStillValid(data_buffer.data()))
            {
                const unsigned len = BinaryToTextEncoder::encode(data_buffer.data(), size, output_ptr);
                return {len, SampleResult::SampledSuccessfully};
            }
        }

        if (isCacheStillValid())
        {
            *output_ptr = '\0';
            return {0, SampleResult::NothingToSample};
        }

        rebuildCache();
        const unsigned len = generateVariableList(output_ptr);
        return {len, SampleResult::VariableListUpdated};
    }

    /**
     * Add one variable to the tracing set.
     * If such variable does not exist at the moment, it will be traced once it appeared in the registry.
     * Variables cannot be removed individually.
     * @param name      name of the variable to trace
     * @return          true if the variable was added successfully or it is already being traced;
     *                  false if the provided name cannot be traced
     */
    bool trace(const ShortName name)
    {
        if (name.isEmpty())
        {
            return false;
        }

        for (auto& n : traced_names_)
        {
            if (name == n)
            {
                return true;        // Already traced, no need to rebuild the cache
            }

            if (n.isEmpty())
            {
                markCacheInvalid();
                n = name;
                return true;        // Added OK
            }
        }

        return false;               // No more space
    }

    /**
     * Stop tracing all variables.
     */
    void stop()
    {
        markCacheInvalid();
        std::fill(traced_names_.begin(), traced_names_.end(), ShortName());
    }
};

/**
 * Returns the list of probe names that are currently registered in the global variable list singleton.
 * WARNING: This function enters a critical section to build the list. Make sure the output iterator is OK with that.
 * @tparam OutIter          deduced automatically
 * @param out_names         contiguous output iterator
 * @param output_capacity   maximum number of names to store via the iterator
 * @return                  number of names written (may differ from the number of registered variables)
 */
template <typename OutIter>
inline unsigned listProbeNames(OutIter out_names, const unsigned output_capacity)
{
    unsigned output_size = 0;

    {
        CriticalSectionLocker locker;
        auto w = ProbeList::getInstance().getHead();
        while ((w != nullptr) && (output_size < output_capacity))
        {
            output_size++;
            *out_names++ = w->getName();
            w = w->getNext();
        }
    }

    return output_size;
}

} // namespace variable_tracer
