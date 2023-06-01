import rospy
from can_msgs.msg import Frame

MAX_DATA_LENGTH = 8
FRAMES_PER_STRING = 3
MAX_STRING_LENGTH = FRAMES_PER_STRING * MAX_DATA_LENGTH


def make_message(arbitration_id, data) -> Frame:
    if len(data) > MAX_DATA_LENGTH:
        raise AssertionError(f"Data size exceeded! {len(data)}/{MAX_DATA_LENGTH}")
    message = Frame()
    message.header.stamp = rospy.Time.now()
    message.id = arbitration_id
    message.dlc = len(data)
    while len(data) < MAX_DATA_LENGTH:
        data.append(0x00)
    message.data = data
    return message

def _string_to_bytes(text):
    return [ord(character) for character in text]

def _string_to_bytelists(text):
    text = text.ljust(MAX_STRING_LENGTH, '\x00')
    return [
        _string_to_bytes(text[i*MAX_DATA_LENGTH:(i+1)*MAX_DATA_LENGTH])
        for i in range(FRAMES_PER_STRING)]

def _bytelists_to_messages(arbitration_ids, text):
    bytelists = _string_to_bytelists(text)
    return [
        make_message(id, bytes)
        for bytes, id in zip(bytelists, arbitration_ids)]
    
def make_string_messages(arbitration_ids, message):
    if len(arbitration_ids) != FRAMES_PER_STRING:
        raise AssertionError(
            f"Must provide {FRAMES_PER_STRING} arbitration IDs. {len(arbitration_ids)} given.")
    if len(message) > MAX_STRING_LENGTH:
        raise AssertionError(
            f"String length limit exceeded! {len(message)}/{MAX_STRING_LENGTH}")
    return _bytelists_to_messages(arbitration_ids, message)

def bits_to_int(bits):
    return int(''.join(map(str, map(int, bits))), 2)