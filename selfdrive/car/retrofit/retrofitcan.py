def toyota_checksum(addr, dat, len):
  cksum = 0
  for i in dat:
    cksum += i
  cksum += (addr & 0xFF) + (addr >> 8) + len
  cksum = cksum & 0xFF
  return cksum

def create_toyota_steer_command(packer, steer, steer_req):
# creates a toyota LKA steer command for retrofit Corolla EPS
  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "SET_ME_1": 1,
  }

  # must manually create the can structure again for checksum on ocelot
  dat1 = (values["STEER_REQUEST"] & 1) | ((values["TOYOTA_COUNTER"] << 1) & 0x3F) | ((values["SET_ME_1"] << 7) & 1)
  dat2 = (values["STEER_TORQUE_CMD"] >> 8) & 0xFF
  dat3 = (values["STEER_TORQUE_CMD"]) & 0xFF
  dat = [dat1,dat2,dat3]

  values["TOYOTA_CHECKSUM"] = toyota_checksum(0x2E4, dat, 5)

  return packer.make_can_msg("STEERING_LKA", 0, values)

def create_steer_command(packer, steer, mode, raw_cnt):
  """Creates a CAN message for the Seb Smith EPAS Steer Command."""

  values = {
    "STEER_MODE": mode,
    "REQUESTED_STEER_TORQUE": steer,
    "COUNTER": raw_cnt,
  }
  return packer.make_can_msg("OCELOT_STEERING_COMMAND", 2, values)

def create_gas_command(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  return packer.make_can_msg("OCELOT_PEDAL_GAS_COMMAND", 2, values)

def create_brake_cmd(packer, enabled, brake, raw_cnt):
  values = {
    "BRAKE_POSITION_COMMAND" : 0,
    "BRAKE_RELATIVE_COMMAND": brake * 252,
    "BRAKE_MODE": enabled,
    "COUNTER" : raw_cnt,
  }
  return packer.make_can_msg("OCELOT_BRAKE_COMMAND", 2, values)