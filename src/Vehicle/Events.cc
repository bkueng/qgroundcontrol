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


ReceiveProtocol::ReceiveProtocol(const EventMetadata& event_metadata, const Callbacks& callbacks,
        uint8_t our_system_id, uint8_t our_component_id,
        uint8_t system_id, uint8_t component_id)
    : _event_metadata(event_metadata),
      _callbacks(callbacks), _our_system_id(our_system_id), _our_component_id(our_component_id),
    _system_id(system_id), _component_id(component_id)
{
}

void ReceiveProtocol::processMessage(const mavlink_message_t& msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_EVENT:
        _handleEvent(msg);
        break;
    case MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE:
        _handleCurrentEventSequence(msg);
        break;
    case MAVLINK_MSG_ID_EVENT_ERROR:
        _handleEventError(msg);
        break;
    }
}
void ReceiveProtocol::_checkTimestampReset(uint32_t timestamp)
{
    if (_last_timestamp_ms == 0) {
        _last_timestamp_ms = timestamp;
    }
    // detect vehicle reboot based on timestamp with some margin and conservative wrap-around handling
    // (in case we missed the current sequence with the reset flag set)
    if (timestamp + 10000 < _last_timestamp_ms && _last_timestamp_ms < UINT32_MAX - 60000) {
        EVENT_DEBUG("sequence reset based on timestamp\n");
        _has_sequence = false;
        _has_current_sequence = false;
    }
}

void ReceiveProtocol::_handleEvent(const mavlink_message_t& message)
{
    mavlink_event_t event_msg;
    mavlink_msg_event_decode(&message, &event_msg);

    if (_component_id != message.compid) {
        // If this happens, the ReceiveProtocol instance is used wrong
        EVENT_DEBUG("got unexpeced component id (%i != %i)", _component_id, message.compid);
        return;
    }

    // check for vehicle reboot (resets the sequence if necessary)
    _checkTimestampReset(event_msg.time_boot_ms);

    if (!_has_sequence) {
        _has_sequence = true;
        _latest_sequence = event_msg.sequence - 1;
    }

    EVENT_DEBUG("Incoming event: last seq=%i, msg seq=%i\n", _latest_sequence, event_msg.sequence);

    switch (_compareSequence(_latest_sequence + 1, event_msg.sequence)) {
        case SequenceComparison::older: // duplicate: discard
            EVENT_DEBUG("Dropping duplicate event");
            return;
        case SequenceComparison::equal: // all good
            _latest_sequence = event_msg.sequence;
            break;
        case SequenceComparison::newer: // dropped events: re-request expected event
            _requestEvent(_latest_sequence + 1);
            // TODO: buffer this event for later processing? For now we just drop it and we'll re-request it later
            return;
    }
    _last_timestamp_ms = event_msg.time_boot_ms;

    // TODO: reset timer

    // need to request more events?
    if (_has_current_sequence) {
        if (_compareSequence(_latest_sequence, _latest_current_sequence) == SequenceComparison::newer) {
            _requestEvent(_latest_sequence + 1);
        }
    }

    // ignore events that are not for us
    if (event_msg.destination_component != _our_component_id && event_msg.destination_component != MAV_COMP_ID_ALL) {
        EVENT_DEBUG("Ignoring event not for us (comp id: %i != %i)\n",
                event_msg.destination_component, _our_component_id);
        return;
    }

    // find metadata & handle the event
    auto event_data = _event_metadata.events.find(event_msg.id);
    if (event_data == _event_metadata.events.end()) {
        _callbacks.handle_unknown_event(event_msg.id);
    } else {
        Event event(_event_metadata.enums, event_data->second, event_msg.time_boot_ms,
                _component_id, event_msg.arguments);
        _callbacks.handle_event(event);
    }

}

ReceiveProtocol::SequenceComparison ReceiveProtocol::_compareSequence(
        uint16_t expected_sequence, uint16_t incoming_sequence)
{
    if (expected_sequence == incoming_sequence) {
        return SequenceComparison::equal;
    }

    // this handles warp-arounds correctly
    const uint16_t diff = incoming_sequence - expected_sequence;
    if (diff > UINT16_MAX / 2) {
        return SequenceComparison::older;
    }
    return SequenceComparison::newer;
}

void ReceiveProtocol::_requestEvent(uint16_t sequence)
{
    mavlink_request_event_t msg;
    msg.target_system = _system_id;
    msg.target_component = _component_id;
    msg.sequence = sequence;

    EVENT_DEBUG("requesting seq %i\n", sequence);

    _callbacks.send_request_event_message(msg);
    // TODO: start a timer (or reset existing timer): 100ms timeout and call _requestEvent again
    // -> calling _handleEvent or _handleEventError stops timer

}

void ReceiveProtocol::_handleCurrentEventSequence(const mavlink_message_t& message)
{
    mavlink_current_event_sequence_t event_sequence;
    mavlink_msg_current_event_sequence_decode(&message, &event_sequence);

    if (event_sequence.flags & MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET) {
        EVENT_DEBUG("current sequence: reset flag set\n");
        _has_sequence = false;
    }
    if (!_has_sequence) {
        _has_sequence = true;
        _latest_sequence = event_sequence.sequence;
    }

    if (_compareSequence(_latest_sequence, event_sequence.sequence) == SequenceComparison::newer) {
        _requestEvent(_latest_sequence + 1);
    }
    _has_current_sequence = true;
    _latest_current_sequence = event_sequence.sequence;
}

void ReceiveProtocol::_handleEventError(const mavlink_message_t& message)
{
    mavlink_event_error_t event_error;
    mavlink_msg_event_error_decode(&message, &event_error);

    if (event_error.target_system != _our_system_id || event_error.target_component != _our_component_id) {
        return;
    }

    if (_compareSequence(_latest_sequence + 1, event_error.sequence) != SequenceComparison::equal) {
        // not a response to our requested sequence number, or we already got the event meanwhile
        return;
    }

    // here we know that we dropped one or more events
    uint16_t num_events_lost = event_error.sequence_oldest_available - _latest_sequence - 1;
    _callbacks.error(num_events_lost);

    _latest_sequence = event_error.sequence_oldest_available - 1;
    _requestEvent(_latest_sequence + 1);
}

} /* namespace events */

