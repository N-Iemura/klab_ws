import odrive
from odrive.enums import *

# Connect to the ODrive
odrv = odrive.find_any()

# Print all attributes of odrv
print(dir(odrv))