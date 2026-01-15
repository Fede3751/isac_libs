import json
from enum import Enum, auto

class MsgType(Enum):

    """
        Class structure which defines the type of messages used by the MEC application.
    """

    # Message sent by agent to ping relays
    HELLO = auto()
    
    # Response sent by relays to an hello message
    HELLO_REPLY = auto()
    
    # Request by an agent for session data
    # (Agent -> Relay)
    SESSION_DATA_AGENT_REQ = auto()

    # Request by an antenna relay for a request of session data
    # (Relay -> Edge Device)
    SESSION_DATA_RELAY_REQ = auto()

    # Response by the edge device for session data, to a relay
    # (Edge Device -> Relay)
    SESSION_DATA_RELAY_CONTENT = auto()

    # Response by the antenna relay for session data, to the agent user
    # (Relay -> Agent)
    SESSION_DATA_AGENT_CONTENT = auto()

    # Message advertised by an edge device when the cache fails to retrieve content.
    # (Edge -> ApplicationController). Not actually bridged in the simulation,
    # just passed inside ROS to evaluate results.
    # We don't simulate actual application data in this context, so there is no data
    # passing between Application Controller and Edge Device.
    EDGE_DEVICE_CACHE_MISS = auto()
    EDGE_DEVICE_CACHE_HIT = auto()

    # Messages used by the relays to find the associated edge device.
    # Differently from an HELLO messages, these protocols are negotiated only at the beginning
    # of the execution, but could also be reused based on application behavior (i.e. moving
    # relays/edge devices).
    # When a reply is received, the rely uses it to check if the current saved edge device has a 
    # better signal, and replaced eventually by the new one
    RELAY_NEGOTIATE_EDGE_DEVICE_REQ = auto()
    RELAY_NEGOTIATE_EDGE_DEVICE_REPLY = auto()

    # Message used by the relays to update telemetry info of an Agent after an HELLO message.
    # (Relay -> Edge Device)
    RELAY_UPDATE_AGENT_TELEMETRY = auto()


class MsgPayload():

    def __init__(self, msg_type: MsgType, msg_content: str = "", msg_dest: str = ""):
        self.msg_type = msg_type
        self.msg_content = msg_content
        self.msg_dest = msg_dest

    @classmethod
    def from_json(cls, msg_json: str):

        msg_struct = json.loads(msg_json)

        return cls(
            msg_type=MsgType[msg_struct.get("type", "")],
            msg_content=msg_struct.get("content", ""),
            msg_dest=msg_struct.get("dest", "")
        )
    
    def to_msg(self):
        return str(self)

    def __str__(self) -> str:
        return json.dumps({
            "type": self.msg_type.name,
            "dest": self.msg_dest,
            "content": self.msg_content
        })