# et732_logger
Code to run on Dangerous Prototypes Web Platform that decodes signal from Maverick ET732 Wireless Thermometer and sends the data to a web server.

Quick and dirty, uses barely-modified example files.  Verified functional with Microchip C30 compiler v3.30c.  Project files are for MPLAB IDE v8.92.

Note: due to licensing issues, I'm not uploading the Microchip Application Libraries - which include the TCP/IP stack.  They're available for free at the Microchip website, and this project was built with the 2011-06-02 version.  The project references it at "C:\Microchip Solutions v2011-06-02".

Functions to decode ET732 data are in MainDemo.c, and data to send to web server is located in GenericTCPClient.c.