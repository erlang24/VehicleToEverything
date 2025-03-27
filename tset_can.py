import can
bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=500000)
seat_msg = can.Message(arbitration_id=0x181,
                        data=[0x01, 0x5e, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0],
                        is_extended_id=False)
bus.send(seat_msg)
bus.shutdown()