/****************************************************************************
 *
 *   (c) 2019 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "Events.h"
#include "MAVLinkProtocol.h"
#include <sstream>
#include <iomanip>

#if 1 // set to 1 to enable debug printf's
#define EVENT_DEBUG printf
#else
#define EVENT_DEBUG(...)
#endif

namespace events {

Event::Event(const Enums& enums, const EventStaticData& data, uint32_t timestamp,
        uint8_t component_id, uint8_t arguments[arguments_max_len])
    : _timestamp(timestamp), _component_id(component_id), _data(data), _enums(enums)
{
    int offset = 0;
    for (const auto& argument : _data.arguments) {
        Argument converted_argument;
        converted_argument.type = argument.type;
        converted_argument.num_decimals = argument.num_decimals;
        converted_argument.enum_index = argument.enum_index;
        int size = 0;
        void* ptr = nullptr;
        switch (argument.type) {
        case ArgumentType::uint8_t_type:
        case ArgumentType::int8_t_type:
            ptr = &converted_argument.data.val_uint8_t;
            size = sizeof(uint8_t);
            break;
        case ArgumentType::uint16_t_type:
        case ArgumentType::int16_t_type:
            ptr = &converted_argument.data.val_uint16_t;
            size = sizeof(uint16_t);
            break;
        case ArgumentType::uint32_t_type:
        case ArgumentType::int32_t_type:
            ptr = &converted_argument.data.val_uint32_t;
            size = sizeof(uint32_t);
            break;
        case ArgumentType::float_type:
            ptr = &converted_argument.data.val_float;
            size = sizeof(float);
            break;
        }
        if (ptr != nullptr && offset + size <= arguments_max_len) {
            // Assuming little endian here
            memcpy(ptr, &arguments[offset], size);
            offset += size;
            _arguments.push_back(converted_argument);
        }
    }
}

std::string Event::message() const
{
    // TODO: process template:
    // - filter by autopilot -> need to get that information
    // - url's & params?
    // - conditions: text that is only added when an argument has a certain value (also: bitfields)
    std::string message = _data.message_template;
    replaceArguments(message);
    return message;
}

void Event::replaceArguments(std::string& message) const
{
    // replace "{1}" with the first argument, and so on
    for (unsigned i = 0; i < _arguments.size(); ++i) {
        std::ostringstream argument;

        // is it an enum?
        int enum_index = _arguments[i].enum_index;
        if (enum_index >= 0 && enum_index < (int)_enums.size()) {
            int enum_value = 0;
            switch(_arguments[i].type) {
                case ArgumentType::uint8_t_type: enum_value = _arguments[i].data.val_uint8_t; break;
                case ArgumentType::int8_t_type:  enum_value = _arguments[i].data.val_int8_t; break;
                case ArgumentType::uint16_t_type: enum_value = _arguments[i].data.val_uint16_t; break;
                case ArgumentType::int16_t_type:  enum_value = _arguments[i].data.val_int16_t; break;
                case ArgumentType::uint32_t_type: enum_value = _arguments[i].data.val_uint32_t; break;
                case ArgumentType::int32_t_type:  enum_value = _arguments[i].data.val_int32_t; break;
                case ArgumentType::float_type: enum_value = _arguments[i].data.val_float; break;
            }
            auto enum_description = _enums[enum_index].descriptions.find(enum_value);
            if (enum_description != _enums[enum_index].descriptions.end()) {
                argument << enum_description->second;
            } else {
                argument << "(unknown)";
            }

        } else {
            switch(_arguments[i].type) {
                case ArgumentType::uint8_t_type: argument << _arguments[i].data.val_uint8_t; break;
                case ArgumentType::int8_t_type:  argument << _arguments[i].data.val_int8_t; break;
                case ArgumentType::uint16_t_type: argument << _arguments[i].data.val_uint16_t; break;
                case ArgumentType::int16_t_type:  argument << _arguments[i].data.val_int16_t; break;
                case ArgumentType::uint32_t_type: argument << _arguments[i].data.val_uint32_t; break;
                case ArgumentType::int32_t_type:  argument << _arguments[i].data.val_int32_t; break;
                case ArgumentType::float_type:
                    argument << std::fixed << std::setprecision(_arguments[i].num_decimals) << _arguments[i].data.val_float;
                    break;
            }
        }
        std::ostringstream search;
        search << '{' << i + 1 << '}';
        findAndReplaceAll(message, search.str(), argument.str());
    }
}


void Event::findAndReplaceAll(std::string& data, const std::string& search, const std::string& replace)
{
    size_t pos = data.find(search);
    while (pos != std::string::npos) {
        data.replace(pos, search.size(), replace);
        pos = data.find(search, pos + replace.size());
    }
}

std::string Event::description() const
{
    // TODO: process template
    return _data.description_template;
}


} /* namespace events */

