from KSN770ScriptEngine import *
import datetime
import time

test = script( __doc__ )

#################
## DEFINITIONS ##
#################
defaultOrigin      = [ 0, "LSZP", 47.0903, 7.2908 ]
defaultDestination = [ 0, "LSZB", 46.9122, 7.4992 ]


defaultOrigin2      = [ 0, "LSZB", 46.9122, 7.4992 ]
defaultDestination2 = [ 0, "LSZP", 47.0903, 7.2908 ]
    

def getDestWpt(destWpt = defaultDestination):
  fpBuffer = test.getFlightPlanBuffer( SECONDARY_FP, WAYPOINT_LIST )
  
  if fpBuffer:
    for record in fpBuffer:
      if record[ 0 ] == WAYPOINT_LIST:
        lastWptArr = record[ -8 : -4 ]
        
        # Return the dest wpt.
        return lastWptArr[ 3 ] | ( lastWptArr[ 2 ] << 8 ) | ( lastWptArr[ 1 ] << 16 ) | ( lastWptArr[ 0 ] << 24 )
        
  return False
  
def setNewOrigin( originWpt = defaultOrigin ):
  # Clear the GF Command Modification list.
  test.log( "Setting a new origin." )
  #deleteFp()
  
  test.log( "Inserting origin waypoint." )
  test.insertWptRequest( SECONDARY_FP, originWpt, ORIGIN_WPT )
  results = test.expectInsertWptResponse()

  test.verifyBool( int( results ) == RESP_STATUS_SUCCESS, "Verify that the origin waypoint insertion was successful" )
  test.waitFpEditComplete()

  
def setNewFlightPlan( originWpt = defaultOrigin, destWpt = defaultDestination ):
  setNewOrigin( originWpt )
  
  test.log( "Inserting destination waypoint." )
  test.insertWptRequest( SECONDARY_FP, destWpt, DEST_WPT )
  results = test.expectInsertWptResponse()

  test.verifyBool( int( results ) == RESP_STATUS_SUCCESS, "Verify that the destination waypoint insertion was successful" )
  test.waitFpEditComplete()
  
  return getDestWpt()
  
def deleteFp( fpln ):
  test.clearGfCommandModifiers()
  test.log( "Deleting the current flight plan." )
  # Clear out existing flight plan.
  test.deleteFpRequest( fpln )
  
  ##########################################################
def getActiveDestWpt(destWpt = defaultDestination2):
  fpBuffer = test.getFlightPlanBuffer( ACTIVE_FP, WAYPOINT_LIST )
  
  if fpBuffer:
    for record in fpBuffer:
      if record[ 0 ] == WAYPOINT_LIST:
        lastWptArr = record[ -8 : -4 ]
        
        # Return the dest wpt.
        return lastWptArr[ 3 ] | ( lastWptArr[ 2 ] << 8 ) | ( lastWptArr[ 1 ] << 16 ) | ( lastWptArr[ 0 ] << 24 )
        
  return False
  
def setNewActiveOrigin( originWpt = defaultOrigin2 ):
  # Clear the GF Command Modification list.
  test.log( "Setting a new origin." )
  #deleteFp()
  
  test.log( "Inserting origin waypoint." )
  test.insertWptRequest( ACTIVE_FP, originWpt, ORIGIN_WPT )
  results = test.expectInsertWptResponse()

  test.verifyBool( int( results ) == RESP_STATUS_SUCCESS, "Verify that the origin waypoint insertion was successful" )
  test.waitFpEditComplete()

  
def setNewActiveFlightPlan( originWpt = defaultOrigin2, destWpt = defaultDestination2 ):
  setNewActiveOrigin( originWpt )
  
  test.log( "Inserting destination waypoint." )
  test.insertWptRequest( ACTIVE_FP, destWpt, DEST_WPT )
  results = test.expectInsertWptResponse()

  test.verifyBool( int( results ) == RESP_STATUS_SUCCESS, "Verify that the destination waypoint insertion was successful" )
  test.waitFpEditComplete()
  
  return getDestWpt()
  ########################################################################
fasFromDBStatus = 0

########################
## INITIAL CONDITIONS ##
########################
test.log("INITIAL CONDITIONS")

# 1. Perform a cold start.
test.enableACSim()
test.ACSim.FASSelect.enable()

# 2. Enable aircraft simulation.
test.ACSim.enableNavigation( 47.0903, 7.2908, 5000, 140, 0 )
#test.verifyBool( ( test.ACSim.FASSelect.fas_select == 0 ), "Verify that fasFromDBStatus == 0" )

################
## ACTION-010 ##
################
test.log( "ACTION-010" )


#Deletes any old flight plan still in the KSN
deleteFp(ACTIVE_FP)
deleteFp(SECONDARY_FP)
test.waitFpEditComplete()

# 3. Create a new flight plan with the origin being set to LSZP, and the destination being set to LSZB.
setNewFlightPlan()



# 4. Enter an arrival procedure with a procedure turn.
test.arrivalRequest( SECONDARY_FP, "LSZB" )
test.verifyBool( test.expectArrivalResponse()[0] is RESP_STATUS_SUCCESS, "Verify that the response status is RESP_STATUS_SUCCESS." )

# 5. Set the approach procedure to RNAV-14.
test.setArrivalApprRequest( SECONDARY_FP, 'RNAV 14' )
resp = test.expectSetArrivalApprResponse()

# 6. Set the approach for BIRKI.
test.setArrivalApprTransRequest( SECONDARY_FP, 'BIRKI' )
resp = test.expectSetArrivalApprTransResponse()

# 7. Send the activate Arrival request.
test.activateArrivalRequest( SECONDARY_FP )
resp = test.expectActivateArrivalResponse()

# 8. Wait for the flight plan edit to complete.
test.waitFpEditComplete()

# 9. Remove any discontinuities.
test.log( "Removing discontinuities" )
fpBuffer = test.getFlightPlanBuffer( SECONDARY_FP, WAYPOINT )

if fpBuffer:
    for record in fpBuffer:
      if record[ 0 ] == WAYPOINT:
        IdentDisplayCode = record[ 128 ]
        ID = record[ 119 : 123 ]
        uniqueWaypointId = ID[ 3 ] | ( ID[ 2 ] << 8 ) | ( ID[ 1 ] << 16 ) | ( ID[ 0 ]  << 24 )
        NameBytes = record[5:14]
        Name = ""
        for val in NameBytes:
            if val == 0:
              break
            Name += chr(val)        
        if IdentDisplayCode == 4:
            test.deleteWptRequest( SECONDARY_FP, uniqueWaypointId )
            results = test.expectDeleteWptResponse()
            test.verifyBool( results is RESP_STATUS_SUCCESS, "Verify that the response status is RESP_STAT_SUCCESS" )
            
            
################
## VERIFY-010 ##
################
test.log( "VERIFY-010" )

test.verifyBool( ( fasFromDBStatus == 0 ), "Verify that fasFromDBStatus == False" )

test.copyFplnRequest( SECONDARY_FP, ACTIVE_FP )

test.waitFpEditComplete()

test.wait(5)




fasFromDBStatus = 0


while fasFromDBStatus == 0:
    fasFromDBStatus = test.ACSim.FASSelect.fas_select
    test.wait(0.5)


test.verifyBool( ( fasFromDBStatus == True ), "Verify that fasFromDBStatus == True" )


deleteFp(ACTIVE_FP)
deleteFp(SECONDARY_FP)
test.wait(35)
test.waitFpEditComplete()

test.wait(5)

################
## ACTION-020 ##
################
test.log( "ACTION-020" )

#1. Creates an active flight plan.
setNewActiveFlightPlan()
test.wait(10)

# 2. Create a new flight plan with the origin being set to LSZP, and the destination being set to LSZB.
setNewFlightPlan()

# 3. Enter an arrival procedure with a procedure turn.
test.arrivalRequest( SECONDARY_FP, "LSZB" )
test.verifyBool( test.expectArrivalResponse()[0] is RESP_STATUS_SUCCESS, "Verify that the response status is RESP_STATUS_SUCCESS." )

# 4. Set the approach procedure to RNAV-14.
test.setArrivalApprRequest( SECONDARY_FP, 'RNAV 14' )
resp = test.expectSetArrivalApprResponse()

# 5. Set the approach for BIRKI.
test.setArrivalApprTransRequest( SECONDARY_FP, 'BIRKI' )
resp = test.expectSetArrivalApprTransResponse()

# 6. Send the activate Arrival request.
test.activateArrivalRequest( SECONDARY_FP )
resp = test.expectActivateArrivalResponse()

# 7. Wait for the flight plan edit to complete.
test.waitFpEditComplete()

# 8. Remove any discontinuities.
test.log( "Removing discontinuities" )
fpBuffer = test.getFlightPlanBuffer( SECONDARY_FP, WAYPOINT )

if fpBuffer:
    for record in fpBuffer:
      if record[ 0 ] == WAYPOINT:
        IdentDisplayCode = record[ 128 ]
        ID = record[ 119 : 123 ]
        uniqueWaypointId = ID[ 3 ] | ( ID[ 2 ] << 8 ) | ( ID[ 1 ] << 16 ) | ( ID[ 0 ]  << 24 )
        NameBytes = record[5:14]
        Name = ""
        for val in NameBytes:
            if val == 0:
              break
            Name += chr(val)        
        if IdentDisplayCode == 4:
            test.deleteWptRequest( SECONDARY_FP, uniqueWaypointId )
            results = test.expectDeleteWptResponse()
            test.verifyBool( results is RESP_STATUS_SUCCESS, "Verify that the response status is RESP_STAT_SUCCESS" )
            
            
################
## VERIFY-020 ##
################
test.log( "VERIFY-020" )
test.verifyBool( ( test.ACSim.FASSelect.fas_select == 0 ), "Verify that fasFromDBStatus == False" )

test.copyFplnRequest( SECONDARY_FP, ACTIVE_FP )

test.waitFpEditComplete()
test.wait(6)



test.verifyBool( ( test.ACSim.FASSelect.fas_select == 1 ), "Verify that fasFromDBStatus == True" )

###################
## END OF SCRIPT ##
###################
test.endScript()
