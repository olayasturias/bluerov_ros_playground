#!/usr/bin/env python

xbox = {
    "axis" :{
      "HR" : 3,
      "VR" : 4,
      "HL" : 0,
      "VL" : 1
      # "padL"   : ,
      # "padR"   : ,
      # "padU"   : ,
      # "padD"   : ,
    }

    # "button":{
    #     "A" :
    #     "B" :
    #     "X" :
    #     "Y" :
    # }

}

FSi6 = {
    "axis" : {
        "HR" : 0,
        "VR" : 1,
        "HL" : 2,
        "VL" : 3,
        "VRA": 4,
        "VRB": 5
    },

    "buttons" : {
        "SWA_down"   : 0,
        "SWA_up"     : 1,
        "SWB_down"   : 2,
        "SWB_up"     : 3,
        "SWC_center" : 4,
        "SWC_down"   : 5,
        "SWC_up"     : 6,
        "SWD_down"   : 7,
        "SWD_up"     : 8

    }
}


joystick = {
  "xbox" : xbox,
  "FSi6" : FSi6
}

axis = {
    "for-backwards": 4,
    "updown"       : 0,
    "rotz"         : 3,
    "roty"         : 1,
    "unknown"      : 5,
    "cam"          : 6 # o 7?
}
