from KSN770ScriptEngine import *
import datetime
import time

test = script( __doc__ )

#################
## DEFINITIONS ##
#################

defaultOrigin      = [ 0, "KPHX", 33.4342, -112.0117 ]#Verify coordinates
defaultDestination = [ 0, "KDVT", 33.6883, -112.0825 ]#Verify coordinates



def getDestWpt(destWpt = defaultDestination):
  fpBuffer = test.getFlightPlanBuffer( ACTIVE_FP, WAYPOINT_LIST )
  
  if fpBuffer:
    for record in fpBuffer:
      if record[ 0 ] == WAYPOINT_LIST:
        lastWptArr = record[ -8 : -4 ]
        
        
        return lastWptArr[ 3 ] | ( lastWptArr[ 2 ] << 8 ) | ( lastWptArr[ 1 ] << 16 ) | ( lastWptArr[ 0 ] << 24 )
        
  return False
  
def setNewOrigin( originWpt = defaultOrigin ):
  
  test.log( "Setting a new origin." )
  
  
  test.log( "Inserting origin waypoint." )
  test.insertWptRequest( ACTIVE_FP, originWpt, ORIGIN_WPT )
  results = test.expectInsertWptResponse()

  test.verifyBool( int( results ) == RESP_STATUS_SUCCESS, "Verify that the origin waypoint insertion was successful" )
  test.waitFpEditComplete()

  
def setNewFlightPlan( originWpt = defaultOrigin, destWpt = defaultDestination ):
  setNewOrigin( originWpt )
  
  test.log( "Inserting destination waypoint." )
  test.insertWptRequest( ACTIVE_FP, destWpt, DEST_WPT )
  results = test.expectInsertWptResponse()

  test.verifyBool( int( results ) == RESP_STATUS_SUCCESS, "Verify that the destination waypoint insertion was successful" )
  test.waitFpEditComplete()
  
  return getDestWpt()
  
def deleteFp( fpln ):
  test.clearGfCommandModifiers()
  test.log( "Deleting the current flight plan." )
  # Clear out existing flight plan.
  test.deleteFpRequest( fpln )

########################
## INITIAL CONDITIONS ##
########################
test.log("INITIAL CONDITIONS")

# 1. Perform a cold start.
test.enableACSim()

# 2. Enable aircraft simulation.
test.ACSim.enableNavigation( 33.4342, -112.0117, 5000, 140, 0 )
test.ACSim.FASSelect.enable()

# 3. Enable the IOF_FMS_GPS_FAS_SELECT label.
test.verifyBool( ( test.ACSim.FASSelect.fas_select == 0 ), "Verify that fasFromDBStatus == 0" )

# 4. Set GPS mode to Navigation.
test.ACSim.PxpressPacket31.enable()
test.ACSim.PxpressPacket31.mode = 2

# 5. Set FMS APM Parameter PREF2_SBAS_PROVIDERS to 1 (Use WAAS discrete)
test.startFmsTransaction( 'getPref', [ 1083 ] )
sbas_providers = test.expectFmsProxyMessage( 'getPrefResponse', 80 ) 
test.log( "sbas_providers {}".format(sbas_providers) )

if (sbas_providers != 0x3FFF):
    test.startFmsTransaction( 'setPref', [ 1083, [ 0x3FFF ] ] )
    response = test.expectFmsProxyMessage( 'setPrefResponse', 80 ) 
    test.log( "Response {}".format(response) )
    assert response == ( "OK", 2 )
    test.wait(5)

    # Perform a cold start.
    test.coldStart()


#############
##ACTION-10##
#############
# 1. Verify that the FAS data block has not been sent to the GPS.
test.verifyBool( test.ACSim.FASSelect.fas_select == 0, "Verify that the FAS data block has not been sent to the GPS." )

# 2. Create a new flight plan with the origin being set to LSZP, and the destination being set to LSZB.
setNewFlightPlan()

# 3. Enter an arrival procedure with a procedure turn.
test.arrivalRequest( ACTIVE_FP, "KDVT" )
test.verifyBool( test.expectArrivalResponse()[0] is RESP_STATUS_SUCCESS, "Verify that the response status is RESP_STATUS_SUCCESS." )

# 4. Set the approach procedure to RNAV-07R.
test.setArrivalApprRequest( ACTIVE_FP, 'RNAV 07R' )
resp = test.expectSetArrivalApprResponse()

# 5. Set the approach for BIRKI.
test.setArrivalApprTransRequest( ACTIVE_FP, 'BANYO' )
resp = test.expectSetArrivalApprTransResponse()

# 6. Send the activate Arrival request.
test.activateArrivalRequest( ACTIVE_FP )
resp = test.expectActivateArrivalResponse()

# 7. Wait for the flight plan edit to complete.
test.waitFpEditComplete()

#############
##VERIFY-10##
#############

# 1. Verify that the FAS data block was sent to the GPS.
test.verifyBool( test.ACSim.FASSelect.fas_select == 1, "Verify that the FAS data block was sent to the GPS" )

#############
##ACTION-20##
#############

#Clear the flight plans.
deleteFp(ACTIVE_FP)
#deleteFp(SECONDARY_FP)
test.waitFpEditComplete()


if (sbas_providers != 0x3FFF):
    test.startFmsTransaction( 'setPref', [ 1083, [ 0x3FFE ] ] )
    response = test.expectFmsProxyMessage( 'setPrefResponse', 80 ) 
    test.log( "Response {}".format(response) )
    assert response == ( "OK", 2 )
    test.wait(5)

# 1. Verify that the FAS data block has not been sent to the GPS.
test.verifyBool( test.ACSim.FASSelect.fas_select == 0, "Verify that the FAS data block has not been sent to the GPS." )

# 2. Create a new flight plan with the origin being set to LSZP, and the destination being set to LSZB.
setNewFlightPlan()

# 3. Enter an arrival procedure with a procedure turn.
test.arrivalRequest( ACTIVE_FP, "KDVT" )
test.verifyBool( test.expectArrivalResponse()[0] is RESP_STATUS_SUCCESS, "Verify that the response status is RESP_STATUS_SUCCESS." )

# 4. Set the approach procedure to RNAV-07R.
test.setArrivalApprRequest( ACTIVE_FP, 'RNAV 07R' )
resp = test.expectSetArrivalApprResponse()

# 5. Set the approach for BIRKI.
test.setArrivalApprTransRequest( ACTIVE_FP, 'BANYO' )
resp = test.expectSetArrivalApprTransResponse()

# 6. Send the activate Arrival request.
test.activateArrivalRequest( ACTIVE_FP )
resp = test.expectActivateArrivalResponse()

# 7. Wait for the flight plan edit to complete.
test.waitFpEditComplete()

#############
##VERIFY-20##
#############
test.log('VERIFY-20')

# 1. Verify that the FAS data block was sent to the GPS.
test.verifyBool( test.ACSim.FASSelect.fas_select == 1, "Verify that the FAS data block was sent to the GPS" )

#Resetting the default value for SBAS providers
if (sbas_providers != 0x3FFF):
    test.startFmsTransaction( 'setPref', [ 1083, [sbas_providers]] )
    response1 = test.expectFmsProxyMessage( 'setPrefResponse', 80 )
    test.log("Response {}".format(response1) )
    test.wait( 5 )

    # perform a cold start
    test.coldStart( )

test.endScript()
