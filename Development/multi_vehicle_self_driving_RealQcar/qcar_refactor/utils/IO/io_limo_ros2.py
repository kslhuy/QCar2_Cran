from .io_base import IOBase

class IOLimoROS2(IOBase):
    # No qcar object — publishes to /cmd_vel or similar
    # Different sensor sources
    # steering may not apply (differential drive)
    pass