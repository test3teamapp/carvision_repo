//#define SECRET_SSID1 "AndroidAPBB98" //samsung j5
//#define SECRET_PASS1 "kfqq3453"
#define SECRET_SSID1 "coalajini"    // huawei lite 8
#define SECRET_PASS1 "olatalefta2u"
#define SECRET_SSID2 "MYSERVICE"    // jetson nano hotspot
#define SECRET_PASS2 "olatalefta2u"
//#define SECRET_SSID4 "Wind WiFi x6JpcR"  //home network
//#define SECRET_PASS4 "Onasougamiso1moretime()"

const char SSIDs[][33] = {SECRET_SSID1, SECRET_SSID2};//, SECRET_SSID3, SECRET_SSID4}; // maximum SSID length is 32 characters (reduce array size to save memory if less will suffice)
const char WiFiPasswords[][65] = {SECRET_PASS1, SECRET_PASS2};//, SECRET_PASS3, SECRET_PASS4}; // maximum password length is 64 cxharacters? (reduce array size to save memory if less will suffice)
