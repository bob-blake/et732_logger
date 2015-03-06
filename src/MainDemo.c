// DangerousPrototypes.com web platform demo
// SD card web server
//
//BEFORE YOU COMPILE!!!!
//Due to licensing issues, we can't redistribute the "Microchip Applications Libraries" (TCPIP stack, etc). 
//You can get them from the Microchip website for free: 
//http://www.microchip.com/tcpip
//
//1.Download and install the "Microchip Applications Libraries". 
//   These files install to c:\Microchip Soultions\ by default.
//2.Put the replacement files in this folder into the Microchip MDD HTTP demo 
//   directory (C:\Microchip Solutions\TCPIP MDD Demo App\)
//   Replace 6 files in total: 
//	 Maindemo.c/.h, HardwareProfile.h, TCPIPConfig.h, TCPIP MDD SD Card Demo App-C30.mcw/.mcp
//3.That's it. You've got the latest source and we're compliant with the license.
//4.Depending on the install location you may need to tweak the include paths 
//   under Project->build options. 
//
// See C:\Microchip Solutions\Microchip\Help\ for the MDD Server help document
//
//EXTRA STEPS FOR BOOTLOADER:
//To use this firmware with the dsPIC30 bootloader:
//1. Export the firmware (File->Export)
//2. Set 0x153FE as the end memory address
//3. Save the firmware and bootload as normal

#define THIS_IS_STACK_APPLICATION //define as entry point, include config bit settings
									//also needed by StackTsk.h, so don't change the name

#include "TCPIP Stack/TCPIP.h" //include TCPIP headers
//#include "MDD File System/FSIO.h" //include FAT file system

APP_CONFIG AppConfig;
static void InitHardware(void);
unsigned char UART1RXRdy(void);
unsigned char UART1RX(void);
void UART1TX(char c);
void InitializeUART1(void);
void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void);

#define TMR_TICK        1.62 //us
#define TWO_T_US        500  //us
#define T_US            250  //us

#define TWO_T_TICKS     309
#define T_TICKS         154

#define TWO_T_LOWER     232 //2T - 0.5T
#define TWO_T_UPPER     386 //etc
#define T_LOWER         77
#define T_UPPER         231

#define NUM_BYTES       13
#define NUM_NIBBLES     NUM_BYTES * 2
#define NUM_BITS        NUM_BYTES * 8

volatile BOOL MemInterfaceAttached=TRUE; //HTTP2 server requires this external variable (don't change the name)
										//could be used to detect media, but we just set it to always true
										//be sure to have an SD card in the slot at startup...

char 		 rx_data[NUM_BITS];
unsigned int tmr_cntr=0,rx_cntr=0,packet_cntr=0;
unsigned int rxing=0,waiting=0,error=0,data_received=0;

//Interrupt vector: _IC1Interrupt (IRQ #1)
void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void){

    unsigned int tmr_val;

    IFS0bits.IC1IF = 0; //Clear interrupt
    TMR2 = 0;   		//Reset Timer 2
    
    if(IFS0bits.T3IF == 1){  //Check to see if we had a packet timeout
        packet_cntr = 0;     //Using interrupt flag as overflow indicator
        rxing = 0;
    }
    TMR3 = 0;               //Reset timeout timer
    IFS0bits.T3IF = 0;      //Clear interrupt flag

    while(IC1CONbits.ICBNE == 1){    //While buffer is not empty
        tmr_val = IC1BUF;

        // If there is a packet in progress
        if(rxing == 1){
            if((tmr_val > T_LOWER) && (tmr_val < T_UPPER)){   // If T
                if(waiting == 1){   // Two consecutive T's means the bit stays the same
                    rx_data[rx_cntr] = rx_data[(rx_cntr - 1)];
                    waiting = 0;
                    rx_cntr++;
                }
                else
                    waiting = 1;
            }
            else if((tmr_val > TWO_T_LOWER) && (tmr_val < TWO_T_UPPER)){ //If 2T, that signals a bit change
                if(waiting == 1){    // Only one consecutive T means an error, jump out
                    error = 1;
                    data_received = 1;
                }
                if(rx_data[(rx_cntr - 1)] == 0)
                    rx_data[rx_cntr] = 1;
                else
                    rx_data[rx_cntr] = 0;
                rx_cntr++;
            }
            if(rx_cntr >= NUM_BITS){
                rxing = 0;
                data_received = 1;
            }
        }        
        else if(rxing == 0){
            if((tmr_val > TWO_T_LOWER) && (tmr_val < TWO_T_UPPER)){    // Wait for 2T to start counting
                rxing = 1;
                waiting = 0;
                error = 0;
                rx_cntr = 2;
                packet_cntr++;
                if(IO8_I == 1){
                    rx_data[0] = 1;         // For first receive, we need to determine
                    rx_data[1] = 0;         // this based upon whether edge rose or fell
                }
                else{                        
                    rx_data[0] = 0;
                    rx_data[1] = 1;
                }
            }
        }
   }

    //if(IC1CONbits.ICOV == 1)    //If there was a buffer overflow
    //    IC1CONbits.ICOV = 0;    //Just clear it for now
}

//Calculate probe temperature in celsius
signed int calc_probe_temp(char which_probe, char *rx_parsed)
{
    int i, offset, probe_temp;
    unsigned char   rx_quart[5];
    
    if(which_probe == 1)
        offset = 8;
    else
        offset = 13;

    //Parse data to quaternary
    for(i=0;i<5;i++){
        switch(rx_parsed[i+offset]){
            case 0x05:
                rx_quart[i] = 0;
            break;
            case 0x06:
                rx_quart[i] = 1;
            break;
            case 0x09:
                rx_quart[i] = 2;
            break;
            case 0x0A:
                rx_quart[i] = 3;
            break;
        }
    }
    probe_temp = 0;
    probe_temp += rx_quart[0] * 256;
    probe_temp += rx_quart[1] * 64;
    probe_temp += rx_quart[2] * 16;
    probe_temp += rx_quart[3] * 4;
    probe_temp += rx_quart[4] * 1;
    probe_temp -= 532;
    return probe_temp;
}

signed int c_to_f(signed int c)
{
	return (1.8*c + 32);	
}

void parse_binary_data(char *binary_in, char *hex_out)
{
    int i,j;
    unsigned char temp;
    // Parse binary data into hex nibbles (stored inefficiently in bytes)
    for(i=0;i<NUM_NIBBLES;i++){
        hex_out[i]=0; //initialize to 0
        for(j=0;j<4;j++){
            hex_out[i] <<= 1;
            temp = binary_in[(i*4)+j];
            hex_out[i] = hex_out[i] | temp;
        }
    }
}

unsigned int verify_rx_data(char *rx_1, char *rx_2, char *rx_3, char *rx_4)
{
	int i;
	
	// Make sure ALL packets are the same because we don't know checksum algorithm
	for(i=0;i<NUM_NIBBLES;i++){
	  if(rx_1[i] != rx_2[i])
	      return 0;
	  if(rx_1[i] != rx_3[i])
	      return 0;
	  if(rx_1[i] != rx_4[i])
	      return 0;
	}
	return 1;
}

int main(void){

	static long ticks = 0;
	static long prevIP = 0;
	
	int             data_valid=1;
    char   			rx_parsed_1[NUM_NIBBLES],rx_parsed_2[NUM_NIBBLES];
    char   			rx_parsed_3[NUM_NIBBLES],rx_parsed_4[NUM_NIBBLES];
    signed int      celsius,probe_1,probe_2;
	DWORD			timestamp;
	
	char food_temp[] = "196";
	char bbq_temp[] = "230";

	BOOL FSready=FALSE; //FileSystemInit() returns type BOOL
	
	InitHardware(); //setup hardware

    TickInit();	//setup the tick timer

	//MPFSInit(); //setup MPFS EEPROM file system

	//setup the TCPIP stack config variable
	AppConfig.Flags.bIsDHCPEnabled = TRUE;
	AppConfig.Flags.bInConfigMode = TRUE;
	AppConfig.MyMACAddr.v[0] = MY_DEFAULT_MAC_BYTE1;
	AppConfig.MyMACAddr.v[1] = MY_DEFAULT_MAC_BYTE2;
	AppConfig.MyMACAddr.v[2] = MY_DEFAULT_MAC_BYTE3;
	AppConfig.MyMACAddr.v[3] = MY_DEFAULT_MAC_BYTE4;
	AppConfig.MyMACAddr.v[4] = MY_DEFAULT_MAC_BYTE5;
	AppConfig.MyMACAddr.v[5] = MY_DEFAULT_MAC_BYTE6;
	AppConfig.MyIPAddr.Val = MY_DEFAULT_IP_ADDR_BYTE1 | MY_DEFAULT_IP_ADDR_BYTE2<<8ul | MY_DEFAULT_IP_ADDR_BYTE3<<16ul | MY_DEFAULT_IP_ADDR_BYTE4<<24ul;
	AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
	AppConfig.MyMask.Val = MY_DEFAULT_MASK_BYTE1 | MY_DEFAULT_MASK_BYTE2<<8ul | MY_DEFAULT_MASK_BYTE3<<16ul | MY_DEFAULT_MASK_BYTE4<<24ul;
	AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
	AppConfig.MyGateway.Val = MY_DEFAULT_GATE_BYTE1 | MY_DEFAULT_GATE_BYTE2<<8ul | MY_DEFAULT_GATE_BYTE3<<16ul | MY_DEFAULT_GATE_BYTE4<<24ul;
	AppConfig.PrimaryDNSServer.Val = MY_DEFAULT_PRIMARY_DNS_BYTE1 | MY_DEFAULT_PRIMARY_DNS_BYTE2<<8ul  | MY_DEFAULT_PRIMARY_DNS_BYTE3<<16ul  | MY_DEFAULT_PRIMARY_DNS_BYTE4<<24ul;
	AppConfig.SecondaryDNSServer.Val = MY_DEFAULT_SECONDARY_DNS_BYTE1 | MY_DEFAULT_SECONDARY_DNS_BYTE2<<8ul  | MY_DEFAULT_SECONDARY_DNS_BYTE3<<16ul  | MY_DEFAULT_SECONDARY_DNS_BYTE4<<24ul;

    StackInit();	//setup the stack
    
	while(1){//never ending loop
	    
		if(FSready==FALSE) FSready=FileSystemInit(); //attempt to initialize the FAT SD card library

        if((TickGet() - ticks) >= (TICK_SECOND/2)){ //blink LED
            ticks = TickGet();
            LED0_IO ^= 1;
        }
		
		//service the stack, includes PING, DHCP client, HTTP2 server, etc (see stacktask.c)
        StackTask();   
        StackApplications();

		//service the TCP client
	GenericTCPClient(&food_temp,&bbq_temp,timestamp);
	//GenericTCPClient(&food_temp,&bbq_temp);

		if(data_received == 1){
            data_received = 0;

            // Need to add a timeout to prevent this from getting out of sync
            if(packet_cntr == 1)
                parse_binary_data(rx_data,rx_parsed_1);
            else if(packet_cntr == 2)
                parse_binary_data(rx_data,rx_parsed_2);
            else if(packet_cntr == 3)
                parse_binary_data(rx_data,rx_parsed_3);
            else if(packet_cntr == 4){
                parse_binary_data(rx_data,rx_parsed_4);
                packet_cntr = 0;

				data_valid = verify_rx_data(rx_parsed_1,rx_parsed_2,rx_parsed_3,rx_parsed_4);
                
                if(data_valid == 1){
                    celsius = calc_probe_temp(1,rx_parsed_4);
                   // probe_1 = c_to_f(celsius);
                   	probe_1 = celsius;
                    sprintf(food_temp,"%d",probe_1);

                    celsius = calc_probe_temp(2,rx_parsed_4);
                  //  probe_2 = c_to_f(celsius);
                  	probe_2 = celsius;
                    sprintf(bbq_temp,"%d",probe_2);
                    
                    timestamp = SNTPGetUTCSeconds();
                    // Add SNTPGetUTCSeconds() in TCP Client routine, send to Cosm
                    
                    TCPTimeToSend();
                    data_valid = 0;
                }
                data_valid = 1;
            }
        }

		#ifdef STACK_USE_ANNOUNCE //announce IP address change if enabled
		if(prevIP != AppConfig.MyIPAddr.Val){
			prevIP = AppConfig.MyIPAddr.Val;
			AnnounceIP();
		}
		#endif

	}
}

//configures the PIC hardware
static void InitHardware(void){		
	AD1PCFGL = 0xFFFF; //digital pins

	//setup internal clock for 80MHz/40MIPS
	//7.37/2=3.685*43=158.455/2=79.2275
	CLKDIVbits.PLLPRE=0; // PLLPRE (N2) 0=/2 
	PLLFBD=41; //pll multiplier (M) = +2
	CLKDIVbits.PLLPOST=0;// PLLPOST (N1) 0=/2
    while(!OSCCONbits.LOCK);//wait for PLL ready

	//Configure Timer2
    T2CON = 0x0000;
    T2CON = 0b1000000000100000;  //Timer2 on, 64x prescale (tick every 1.62 us)
    TMR2 = 0x0000;

    //Configure Timer3
    //This is to determine timeout.  Start of preamble to end of each packet, 93ms
    //Packet itself is 51ms
    T3CON = 0x0000;
    T3CON = 0b1000000000110000;     //Timer3 on, 256x prescale (tick every 6.451 us)
    TMR3 = 0x0000;

    //Configure input capture
    IC1CONbits.ICM = 0;         // Disable module while setting up
    IC1CONbits.ICTMR = 1;       // Use timer 2
    IC1CONbits.ICI = 0b00;      // Interrupt on every capture event
    IC1CONbits.ICM = 0b001;     // Capture on every edge, rising and falling

    //Setup interrupts
    IPC0bits.IC1IP = 1; // Setup IC1 interrupt priority level
    IFS0bits.IC1IF = 0; // Clear flag just in case
    IEC0bits.IC1IE = 1; // Enable input capture 1

	//setup LEDs
	LED0_TRIS = 0;
	LED1_TRIS = 0;
	LED2_TRIS = 0;
	LED0_IO=1;
	LED1_IO=1;
	LED2_IO=1;

	//custom pin assignments for our hardware
	// ENC28J60 I/O pins
	//mapping:
	//A2 ETH-INT
	//C2 MISO
	//C1 MOSI
	//C0 CLK
	//B3 CS
	//B2 RST
	//CS and RST pins
	//MISO1 C2/RP18 (input)
	SDI1R_I = 18;			
	//CLK1 C0/RP16 (output)
	RP16_O = SCK1OUT_O; 	
	//MOSI1 C1/RP17 (output)
	RP17_O = SDO1_O;		

    //Assign RP pin for input capture
    IO8_TRIS = 1;   // Set pin direction to input
    IC1R_I = 7;     //assign input capture function to RP7 (IO8/RB7)

	//SD CARD PPS
	//MISO1 B10/RP10 (input)
	SDI2R_I = 10;			
	//CLK1 B11/RP11 (output)
	RP11_O = SCK2OUT_O; 	
	//MOSI1 B12/RP12 (output)
	RP12_O = SDO2_O;
	
/*	//EEPROM PPS
	//MISO2 C8/RP24 (input)
	SDI2R_I = 24;			
	//CLK2 C6/RP22 (output)
	RP22_O = SCK2OUT_O; 	
	//MOSI2 B9/RP9 (output)
	RP9_O = SDO2_O;		
	XEEInit();
*/
	//uart
	//RX PR14 (input)
	U1RXR_I = 14;
	//TX RP15 (output)
	RP15_O=U1TX_O;

	//InitializeUART1();


	//lock PPS
	asm volatile (	"mov #OSCCON,w1 \n"
					"mov #0x46, w2 \n"
					"mov #0x57, w3 \n"
					"mov.b w2,[w1] \n"
					"mov.b w3,[w1] \n"
					"bset OSCCON, #6");
}


//is data available in RX buffer?
//#define UART1RXRdy() U1STAbits.URXDA
unsigned char UART1RXRdy(void){
    return U1STAbits.URXDA;
}

//get a byte from UART
unsigned char UART1RX(void){

    while(U1STAbits.URXDA == 0);
	return U1RXREG;
}

//add byte to buffer, pause if full
//uses PIC 4 byte UART FIFO buffer
void UART1TX(char c){
	U1STAbits.UTXEN = 1; //enable UART TX
	while(U1STAbits.UTXBF == 1); //if buffer is full, wait
    U1TXREG = c;
	while(U1STAbits.TRMT== 0); //wait for UART to empty
	U1STAbits.UTXEN = 0;	//disable TX to avoid backpowering FTDI chip when no USB attached
}

//Initialize the terminal UART
void InitializeUART1(void){
	//setup UART
    U1BRG = 85;//86@80mhz, 85@79.xxx=115200
    U1MODE = 0;
    U1MODEbits.BRGH = 1;
    U1STA = 0;
    U1MODEbits.UARTEN = 1;
    IFS0bits.U1RXIF = 0;
}

//stack overslow interrupt vectors
void _ISR __attribute__((__no_auto_psv__)) _AddressError(void)
{
    Nop();
	Nop();
}
void _ISR __attribute__((__no_auto_psv__)) _StackError(void)
{
    Nop();
	Nop();
}

