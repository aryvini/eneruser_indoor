//IN ORDER TO WORK RENAME THIS FILE TO secrets.h


int status = WL_IDLE_STATUS; // the Wifi radio's status

// char ssid[] = "SSID";      // your network SSID (name)
// char pwd[] = "Password";   // your network password


//Server IP data
IPAddress spanish_server(100, 100, 100, 100);
unsigned int spanish_port = 8888; //Port to transfer data


//Logger metadata
char location_name[12] = "XXXX"; //Chose an easy name
char host_name[10] = "node2"; //host_name leave this