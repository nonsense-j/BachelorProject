# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
any_region, 1

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../ResCap/ResCap.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
find_casualty, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
right = p2, p4
up = p3, p9, p10, p11
down = p8, p12, p13, p14
others = 
nofly = p3, p4, p6, p8
left = p5, p6

Spec: # Specification in structured English
# initial conditions
Env starts with false
you start in left and !nofly with false

# Define robot safety including how to attack
Do any_region if and only if you are sensing find_casualty

