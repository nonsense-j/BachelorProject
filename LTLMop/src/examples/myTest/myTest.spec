# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
alarm, 1

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

CurrentConfigName:
myTest

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
myTest.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
get_up, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = 
hall = p2
room = p1

Spec: # Specification in structured English
# initial conditions
Env starts with false
you start in hall with false

Do alarm if and only if you are sensing get_up
if you are activating alarm then go to room

