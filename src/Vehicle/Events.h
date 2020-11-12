/****************************************************************************
 *
 *   (c) 2019 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include "MAVLinkProtocol.h"

#include <functional>
#include <map>
#include <vector>

namespace events {

enum class LogLevel {
    Emergency = 0,
    Alert = 1,
    Critical = 2,
    Error = 3,
    Warning = 4,
    Notice = 5,
    Info = 6,
    Protocol = 7,

    Disabled = 8,
};

enum class ArgumentType {
    uint8_t_type,
    int8_t_type,
    uint16_t_type,
    int16_t_type,
    uint32_t_type,
    int32_t_type,
    float_type
};

struct EventArgument {
    EventArgument(ArgumentType t, int enum_idx=-1, int decimals=0) : type(t), enum_index(enum_idx),
            num_decimals(decimals) {}

    ArgumentType type;
    int enum_index; ///< -1 if no enum, otherwise index for the enum metadata vector
    int num_decimals; ///< how many decimal digits to print (for floats)
};

/**
 * @class EventStaticData
 * Meta data for a single event, read from an XML. It contains the translated message template
 */
struct EventStaticData {
    uint32_t id;
    std::string name;
    std::string message_template;
    std::string description_template;

    LogLevel log_level;
    std::string group;

    std::vector<EventArgument> arguments;
};

struct Enum {
    std::string name;
    std::map<int, std::string> descriptions;
    // TODO: bitfield?
};

using EventMap = std::map<uint32_t, EventStaticData>;
using Enums = std::vector<Enum>;

struct EventMetadata {
    Enums enums;
    EventMap events;
};

/**
 * @class Event
 */
class Event {
public:
    static constexpr int arguments_max_len = 32; // TODO: get max argument size from somewhere...

    Event(const Enums& enums, const EventStaticData& data, uint32_t timestamp,
            uint8_t component_id, uint8_t arguments[arguments_max_len]);

    uint32_t eventId() const { return _data.id; }

    const std::string& eventName() const { return _data.name; }

    uint8_t componentId() const { return _component_id; }

    std::string message() const;
    std::string description() const;

    LogLevel logLevel() const { return _data.log_level; }
    std::string group() const { return _data.group; }

    uint32_t timestamp() const { return _timestamp; }

    struct Argument {
        union {
            uint8_t val_uint8_t;
            int8_t val_int8_t;
            uint16_t val_uint16_t;
            int16_t val_int16_t;
            uint32_t val_uint32_t;
            int32_t val_int32_t;
            float val_float;
        } data;
        int num_decimals;
        ArgumentType type;
        int enum_index;
    };

    const std::vector<Argument>& arguments() const { return _arguments; }

private:
    void replaceArguments(std::string& message) const;
    static void findAndReplaceAll(std::string& data, const std::string& search, const std::string& replace);

    const uint32_t _timestamp; ///< timestamp since boot in ms
    const uint8_t _component_id;

    std::vector<Argument> _arguments;

    const EventStaticData& _data;
    const Enums& _enums;
};

} /* namespace events */
