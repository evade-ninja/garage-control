void handleRoot();
void handleNarrowCSS();
void handleStatus();
void handleUpdate();
void handleNotFound();
void setup ( void );
void loop ( void );
void handleToggle();
void toggleDoor();
int getDoorState();
void getDoorStateV();
void logEvent(String event);
char* levelToString(int level);
char const* getDoorStateChr();
time_t getNtpTime();
unsigned long sendNTPpacket(IPAddress& address);
void syncTime();

