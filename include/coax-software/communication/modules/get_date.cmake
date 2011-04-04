##############################################################################################
# Author: Christoph Hürzeler Date: 12.11.08
# 
# Get the current date and time and store in CURRENT_DATE variable
##############################################################################################

# Get the current date
#TODO: make this cross-platform compatible
EXECUTE_PROCESS( COMMAND date -u  OUTPUT_VARIABLE CURRENT_DATE )
STRING( REPLACE "\n" "" CURRENT_DATE ${CURRENT_DATE} )
