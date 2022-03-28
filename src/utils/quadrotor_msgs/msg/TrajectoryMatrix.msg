#type
uint8 TYPE_UNKNOWN = 0
uint8 TYPE_POLY    = 1
uint8 TYPE_TIME    = 2

#data structure
Header    header
uint8     type
uint32    id
string    name
uint32[]  format
float64[] data
