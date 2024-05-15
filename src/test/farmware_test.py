import odrive
from odrive.enums import *

# Connect to the ODrive
odrv = odrive.find_any()

# Print the firmware version
print(odrv.fw_version_major)
print(odrv.fw_version_minor)
print(odrv.fw_version_revision)
print(odrv.fw_version_unreleased)