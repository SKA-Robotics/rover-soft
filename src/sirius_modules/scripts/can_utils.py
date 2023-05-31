import can
import time

MAX_DATA_LENGTH = 8
FRAMES_PER_STRING = 3
MAX_STRING_LENGTH = FRAMES_PER_STRING * MAX_DATA_LENGTH

can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'slcan0'
can.rc['bitrate'] = 500000
bus = can.interface.Bus()

def send(arbitration_id, data):
    if len(data) > MAX_DATA_LENGTH:
        raise AssertionError(f"Data size exceeded! {len(data)}/{MAX_DATA_LENGTH}")
    message = can.Message(arbitration_id=arbitration_id,
                       is_extended_id =False,
                       data=data)
    bus.send(message)
    time.sleep(0.005)

def _string_to_bytes(text):
    return [ord(character) for character in text]

def _string_to_bytelists(text):
    text = text.ljust(MAX_STRING_LENGTH, '\x00')
    return [
        _string_to_bytes(text[i*MAX_DATA_LENGTH:(i+1)*MAX_DATA_LENGTH])
        for i in range(FRAMES_PER_STRING)]

def _make_string_messages(arbitration_ids, text):
    bytelists = _string_to_bytelists(text)
    return [
        can.Message(arbitration_id=id, is_extended_id=False, data=bytes)
        for bytes, id in zip(bytelists, arbitration_ids)]
    
def send_string(arbitration_ids, message):
    if len(arbitration_ids) != FRAMES_PER_STRING:
        raise AssertionError(
            f"Must provide {FRAMES_PER_STRING} arbitration IDs. {len(arbitration_ids)} given.")
    if len(message) > MAX_STRING_LENGTH:
        raise AssertionError(
            f"String length limit exceeded! {len(message)}/{MAX_STRING_LENGTH}")
    messages = _make_string_messages(arbitration_ids, message)
    for message in messages:
        bus.send(message)
        time.sleep(0.005)

def bits_to_int(bits):
    return int(''.join(map(str, map(int, bits))), 2)

def close():
    bus.shutdown()