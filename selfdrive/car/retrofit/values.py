import re
from collections import defaultdict
from dataclasses import dataclass, field
from enum import IntFlag, StrEnum
from typing import Dict, List, Set, Union

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import AngleRateLimit, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarInfo, CarParts, CarHarness
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu
MIN_ACC_SPEED = 19. * CV.MPH_TO_MS
PEDAL_TRANSITION = 10. * CV.MPH_TO_MS


class CarControllerParams:
  ACCEL_MAX = 1.5  # m/s2, lower than allowed 2.0 m/s2 for tuning reasons
  ACCEL_MIN = -3.5  # m/s2

  STEER_STEP = 1
  STEER_MAX = 1500
  STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor

  # Lane Tracing Assist (LTA) control limits
  # Assuming a steering ratio of 13.7:
  # Limit to ~2.0 m/s^3 up (7.5 deg/s), ~3.5 m/s^3 down (13 deg/s) at 75 mph
  # Worst case, the low speed limits will allow ~4.0 m/s^3 up (15 deg/s) and ~4.9 m/s^3 down (18 deg/s) at 75 mph,
  # however the EPS has its own internal limits at all speeds which are less than that
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[5, 25], angle_v=[0.3, 0.15])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[5, 25], angle_v=[0.36, 0.26])

  def __init__(self, CP):
    if CP.lateralTuning.which == 'torque':
      self.STEER_DELTA_UP = 15       # 1.0s time to peak torque
      self.STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
    else:
      self.STEER_DELTA_UP = 10       # 1.5s time to peak torque
      self.STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)


class ToyotaFlags(IntFlag):
  HYBRID = 1
  SMART_DSU = 2
  DISABLE_RADAR = 4


class CAR(StrEnum):
  # Toyota
  RSG_OFFROAD = "CUSTOM 4Runner RSG Offroad"


@dataclass
class ToyotaCarInfo(CarInfo):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


CAR_INFO: Dict[str, Union[ToyotaCarInfo, List[ToyotaCarInfo]]] = {
  CAR.RSG_OFFROAD: [
    ToyotaCarInfo("CUSTOM 4Runner RSG Offroad"),
  ],
}

EPS_SCALE = defaultdict(lambda: 73, {CAR.RSG_OFFROAD: 88})

# (addr, cars, bus, 1/freq*100, vl)
STATIC_DSU_MSGS = [
  (0x128, (CAR.RSG_OFFROAD), 1,   3, b'\xf4\x01\x90\x83\x00\x37'),
  (0x141, (CAR.RSG_OFFROAD), 1,   2, b'\x00\x00\x00\x46'),
  (0x160, (CAR.RSG_OFFROAD), 1,   7, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, (CAR.RSG_OFFROAD), 1,   7, b'\x00\x1e\x00\x00\x00\x80\x07'),
  (0x283, (CAR.RSG_OFFROAD), 0,   3, b'\x00\x00\x00\x00\x00\x00\x8c'),
  (0x344, (CAR.RSG_OFFROAD), 0,   5, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, (CAR.RSG_OFFROAD), 0,  20, b'\x00\x00\x00\x80\xfc\x00\x08'),
  (0x366, (CAR.RSG_OFFROAD), 0,  20, b'\x00\x72\x07\xff\x09\xfe\x00'),
  (0x4CB, (CAR.RSG_OFFROAD), 0, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
]


def get_platform_codes(fw_versions: List[bytes]) -> Dict[bytes, Set[bytes]]:
  # Returns sub versions in a dict so comparisons can be made within part-platform-major_version combos
  codes = defaultdict(set)  # Optional[part]-platform-major_version: set of sub_version
  for fw in fw_versions:
    # FW versions returned from UDS queries can return multiple fields/chunks of data (different ECU calibrations, different data?)
    #  and are prefixed with a byte that describes how many chunks of data there are.
    # But FW returned from KWP requires querying of each sub-data id and does not have a length prefix.

    length_code = 1
    length_code_match = FW_LEN_CODE.search(fw)
    if length_code_match is not None:
      length_code = length_code_match.group()[0]
      fw = fw[1:]

    # fw length should be multiple of 16 bytes (per chunk, even if no length code), skip parsing if unexpected length
    if length_code * FW_CHUNK_LEN != len(fw):
      continue

    chunks = [fw[FW_CHUNK_LEN * i:FW_CHUNK_LEN * i + FW_CHUNK_LEN].strip(b'\x00 ') for i in range(length_code)]

    # only first is considered for now since second is commonly shared (TODO: understand that)
    first_chunk = chunks[0]
    if len(first_chunk) == 8:
      # TODO: no part number, but some short chunks have it in subsequent chunks
      fw_match = SHORT_FW_PATTERN.search(first_chunk)
      if fw_match is not None:
        platform, major_version, sub_version = fw_match.groups()
        codes[b'-'.join((platform, major_version))].add(sub_version)

    elif len(first_chunk) == 10:
      fw_match = MEDIUM_FW_PATTERN.search(first_chunk)
      if fw_match is not None:
        part, platform, major_version, sub_version = fw_match.groups()
        codes[b'-'.join((part, platform, major_version))].add(sub_version)

    elif len(first_chunk) == 12:
      fw_match = LONG_FW_PATTERN.search(first_chunk)
      if fw_match is not None:
        part, platform, major_version, sub_version = fw_match.groups()
        codes[b'-'.join((part, platform, major_version))].add(sub_version)

  return dict(codes)


def match_fw_to_car_fuzzy(live_fw_versions) -> Set[str]:
  candidates = set()

  for candidate, fws in FW_VERSIONS.items():
    # Keep track of ECUs which pass all checks (platform codes, within sub-version range)
    valid_found_ecus = set()
    valid_expected_ecus = {ecu[1:] for ecu in fws if ecu[0] in PLATFORM_CODE_ECUS}
    for ecu, expected_versions in fws.items():
      addr = ecu[1:]
      # Only check ECUs expected to have platform codes
      if ecu[0] not in PLATFORM_CODE_ECUS:
        continue

      # Expected platform codes & versions
      expected_platform_codes = get_platform_codes(expected_versions)

      # Found platform codes & versions
      found_platform_codes = get_platform_codes(live_fw_versions.get(addr, set()))

      # Check part number + platform code + major version matches for any found versions
      # Platform codes and major versions change for different physical parts, generation, API, etc.
      # Sub-versions are incremented for minor recalls, do not need to be checked.
      if not any(found_platform_code in expected_platform_codes for found_platform_code in found_platform_codes):
        break

      valid_found_ecus.add(addr)

    # If all live ECUs pass all checks for candidate, add it as a match
    if valid_expected_ecus.issubset(valid_found_ecus):
      candidates.add(candidate)

  return {str(c) for c in (candidates - FUZZY_EXCLUDED_PLATFORMS)}


# Regex patterns for parsing more general platform-specific identifiers from FW versions.
# - Part number: Toyota part number (usually last character needs to be ignored to find a match).
#    Each ECU address has just one part number.
# - Platform: usually multiple codes per an openpilot platform, however this is the least variable and
#    is usually shared across ECUs and model years signifying this describes something about the specific platform.
#    This describes more generational changes (TSS-P vs TSS2), or manufacture region.
# - Major version: second least variable part of the FW version. Seen splitting cars by model year/API such as
#    RAV4 2022/2023 and Avalon. Used to differentiate cars where API has changed slightly, but is not a generational change.
#    It is important to note that these aren't always consecutive, for example:
#    Avalon 2016-18's fwdCamera has these major versions: 01, 03 while 2019 has: 02
# - Sub version: exclusive to major version, but shared with other cars. Should only be used for further filtering.
#    Seen bumped in TSB FW updates, and describes other minor differences.
SHORT_FW_PATTERN = re.compile(b'[A-Z0-9](?P<platform>[A-Z0-9]{2})(?P<major_version>[A-Z0-9]{2})(?P<sub_version>[A-Z0-9]{3})')
MEDIUM_FW_PATTERN = re.compile(b'(?P<part>[A-Z0-9]{5})(?P<platform>[A-Z0-9]{2})(?P<major_version>[A-Z0-9]{1})(?P<sub_version>[A-Z0-9]{2})')
LONG_FW_PATTERN = re.compile(b'(?P<part>[A-Z0-9]{5})(?P<platform>[A-Z0-9]{2})(?P<major_version>[A-Z0-9]{2})(?P<sub_version>[A-Z0-9]{3})')
FW_LEN_CODE = re.compile(b'^[\x01-\x03]')  # highest seen is 3 chunks, 16 bytes each
FW_CHUNK_LEN = 16

# List of ECUs that are most unique across openpilot platforms
# TODO: use hybrid ECU, splits similar ICE and hybrid variants
# - fwdCamera: describes actual features related to ADAS. For example, on the Avalon it describes
#    when TSS-P became standard, whether the car supports stop and go, and whether it's TSS2.
#    On the RAV4, it describes the move to the radar doing ACC, and the use of LTA for lane keeping.
# - abs: differentiates hybrid/ICE on most cars (Corolla TSS2 is an exception)
# - eps: describes lateral API changes for the EPS, such as using LTA for lane keeping and rejecting LKA messages
PLATFORM_CODE_ECUS = [Ecu.fwdCamera, Ecu.abs, Ecu.eps]

# These platforms have at least one platform code for all ECUs shared with another platform.
FUZZY_EXCLUDED_PLATFORMS = {CAR.LEXUS_ES_TSS2, CAR.LEXUS_RX_TSS2}

# Some ECUs that use KWP2000 have their FW versions on non-standard data identifiers.
# Toyota diagnostic software first gets the supported data ids, then queries them one by one.
# For example, sends: 0x1a8800, receives: 0x1a8800010203, queries: 0x1a8801, 0x1a8802, 0x1a8803
TOYOTA_VERSION_REQUEST_KWP = b'\x1a\x88\x01'
TOYOTA_VERSION_RESPONSE_KWP = b'\x5a\x88\x01'

FW_QUERY_CONFIG = FwQueryConfig(
  # TODO: look at data to whitelist new ECUs effectively
  requests=[
    Request(
      [StdQueries.SHORT_TESTER_PRESENT_REQUEST, TOYOTA_VERSION_REQUEST_KWP],
      [StdQueries.SHORT_TESTER_PRESENT_RESPONSE, TOYOTA_VERSION_RESPONSE_KWP],
      whitelist_ecus=[Ecu.fwdCamera, Ecu.fwdRadar, Ecu.dsu, Ecu.abs, Ecu.eps, Ecu.epb, Ecu.telematics,
                      Ecu.srs, Ecu.combinationMeter, Ecu.transmission, Ecu.gateway, Ecu.hvac],
      bus=0,
    ),
    Request(
      [StdQueries.SHORT_TESTER_PRESENT_REQUEST, StdQueries.OBD_VERSION_REQUEST],
      [StdQueries.SHORT_TESTER_PRESENT_RESPONSE, StdQueries.OBD_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine, Ecu.epb, Ecu.telematics, Ecu.hybrid, Ecu.srs, Ecu.combinationMeter, Ecu.transmission,
                      Ecu.gateway, Ecu.hvac],
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.DEFAULT_DIAGNOSTIC_REQUEST, StdQueries.EXTENDED_DIAGNOSTIC_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.DEFAULT_DIAGNOSTIC_RESPONSE, StdQueries.EXTENDED_DIAGNOSTIC_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine, Ecu.fwdRadar, Ecu.fwdCamera, Ecu.abs, Ecu.eps, Ecu.epb, Ecu.telematics,
                      Ecu.hybrid, Ecu.srs, Ecu.combinationMeter, Ecu.transmission, Ecu.gateway, Ecu.hvac],
      bus=0,
    ),
  ],
  non_essential_ecus={
    # FIXME: On some models, abs can sometimes be missing
    Ecu.abs: [CAR.RAV4, CAR.RSG_OFFROAD, CAR.HIGHLANDER, CAR.SIENNA, CAR.LEXUS_IS],
    # On some models, the engine can show on two different addresses
    Ecu.engine: [CAR.CAMRY, CAR.RSG_OFFROAD_TSS2, CAR.CHR, CAR.CHR_TSS2, CAR.LEXUS_IS, CAR.LEXUS_RC,
                 CAR.LEXUS_NX, CAR.LEXUS_NX_TSS2, CAR.LEXUS_RX_TSS2],
  },
  extra_ecus=[
    # All known ECUs on a late-model Toyota vehicle not queried here:
    # Responds to UDS:
    # - HV Battery (0x713, 0x747)
    # - Motor Generator (0x716, 0x724)
    # - 2nd ABS "Brake/EPB" (0x730)
    # Responds to KWP (0x1a8801):
    # - Steering Angle Sensor (0x7b3)
    # - EPS/EMPS (0x7a0, 0x7a1)
    # Responds to KWP (0x1a8881):
    # - Body Control Module ((0x750, 0x40))

    # Hybrid control computer can be on 0x7e2 (KWP) or 0x7d2 (UDS) depending on platform
    (Ecu.hybrid, 0x7e2, None),  # Hybrid Control Assembly & Computer
    # TODO: if these duplicate ECUs always exist together, remove one
    (Ecu.srs, 0x780, None),     # SRS Airbag
    (Ecu.srs, 0x784, None),     # SRS Airbag 2
    # Likely only exists on cars where EPB isn't standard (e.g. Camry, Avalon (/Hybrid))
    # On some cars, EPB is controlled by the ABS module
    (Ecu.epb, 0x750, 0x2c),     # Electronic Parking Brake
    # This isn't accessible on all cars
    (Ecu.gateway, 0x750, 0x5f),
    # On some cars, this only responds to b'\x1a\x88\x81', which is reflected by the b'\x1a\x88\x00' query
    (Ecu.telematics, 0x750, 0xc7),
    # Transmission is combined with engine on some platforms, such as TSS-P RAV4
    (Ecu.transmission, 0x701, None),
    # A few platforms have a tester present response on this address, add to log
    (Ecu.transmission, 0x7e1, None),
    # On some cars, this only responds to b'\x1a\x88\x80'
    (Ecu.combinationMeter, 0x7c0, None),
    (Ecu.hvac, 0x7c4, None),
  ],
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

FW_VERSIONS = {
  CAR.RSG_OFFROAD: {
    # TODO: fingerprint his car
    (Ecu.abs, 0x7b0, None): [
      b'F152607060\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.dsu, 0x791, None): [
      b'881510701300\x00\x00\x00\x00',
      b'881510705100\x00\x00\x00\x00',
      b'881510705200\x00\x00\x00\x00',
    ],
    (Ecu.eps, 0x7a1, None): [
      b'8965B41051\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'\x0230721100\x00\x00\x00\x00\x00\x00\x00\x00A0C01000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x0230721200\x00\x00\x00\x00\x00\x00\x00\x00A0C01000\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x750, 0xf): [
      b'8821F4702000\x00\x00\x00\x00',
      b'8821F4702100\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x750, 0x6d): [
      b'8646F0701100\x00\x00\x00\x00',
      b'8646F0703000\x00\x00\x00\x00',
    ],
  },
}

STEER_THRESHOLD = 100

DBC = {
  CAR.RSG_OFFROAD: dbc_dict('toyota_tundra_2012', 'toyota_adas', 'ocelot_controls'),
}