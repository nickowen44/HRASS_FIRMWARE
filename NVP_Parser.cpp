#include "Arduino.h"
#include "NVP_Parser.h"

char* RFQ;
char* RPW;

void fclass::NVPparser(String NVP_Input) {

  char str[100];  // An array to hold 50 integers
  KeyValuePair pairs[MAX_PAIRS];
  NVP_Input.toCharArray(str, 100);  //convert th input NVP string to a char array
  char* saveptr1;                   // Initialize our save pointers to save the context of tokens
  char* saveptr2;
  int count = 0;     // counter to split the keys and values
  int kv_count = 0;  // count the number of keyvalue pairs

  char* tok = strtok_r(str, "| ", &saveptr1);
  while (tok != NULL) {
    //printf("Token: %s\n", tok);
    char* subtok = strtok_r(tok, "=", &saveptr2);
    while (subtok != NULL) {  // Once we have an Key value pairs, break it down seperating the keys and values
      if (count == 0) {
        //printf("Key: %s\n", subtok);
        strncpy(pairs[kv_count].key, subtok, MAX_KV_LEN);
        count++;
      } else {
        //printf("Value: %s\n", subtok);
        strncpy(pairs[kv_count].value, subtok, MAX_KV_LEN);
      }
      subtok = strtok_r(NULL, "=", &saveptr2);
    }
    count = 0;
    kv_count++;
    tok = strtok_r(NULL, "| ", &saveptr1);
  }

  // Print the results
  for (int i = 0; i < kv_count; i++) {
    //printf("Key: %s, Value: %s\n", pairs[i].key, pairs[i].value);

    if (!strcmp(pairs[i].key, "RFQ")) { RFQ = pairs[i].value; }
    if (!strcmp(pairs[i].key, "RPW")) { RPW = pairs[i].value; }
  }

  // String myString = String(pairs[1].value);
  //String myString = String("hello");

  // return (myString);
}


fclass NVP = fclass();
