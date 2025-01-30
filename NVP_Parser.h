/*
  Key Value Pair Parser

    This code parses pairs of keys and values to global variables 

    example input :
    NVP.NVPparser("var1=red|var2=green|var3=up|var4=down|RPW=50|RFQ=100|time=123443291|key=left");


  Created 30 Jan 2025
  By Nick Owen
  Nuvotion P/L
*/

#ifndef fan_h
#define fan_h

#define MAX_PAIRS 10  // max number of anticipated key value pairs
#define MAX_KV_LEN 100

class fclass {

public:
//Variables
  typedef struct {
    char key[MAX_KV_LEN];
    char value[MAX_KV_LEN];
  } KeyValuePair;


  //Functions
  void NVPparser(String NVP_Input);

  //void nvpmatcher(char key, char value);
};

extern fclass NVP;

//Global NVP variables
extern char *RFQ;  // rain frequency
extern char *RPW;  // rain pulse width

#endif
