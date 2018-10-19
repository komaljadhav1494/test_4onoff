void applicationControl(char switchNumberMSB, char switchNumberLSB, char switchStatus, char SpeedMSB, char SpeedLSB,char ChildLock,char FinalBit){
    int intSwitchStatus=0;
    int intswitchNumberLSB=0;
    intswitchNumberLSB =  switchNumberLSB -'0';
    intSwitchStatus = switchStatus-'0';
    ChildLockBuffer[intswitchNumberLSB]=ChildLock;
   // TX1REG=ChildLockBuffer[switchNumberLSB];
    if(FinalBit == '1')
    {
        TX1REG='G';__delay_ms(1);
        TX1REG=switchStatus;__delay_ms(1);
        TX1REG='0';__delay_ms(1);
        TX1REG=switchNumberLSB;__delay_ms(1);
    }
    switch(switchNumberLSB)
    {
        case '1':
        {
            OUTPUT_RELAY1 = intSwitchStatus;
        }
        break;
        case '2':
        {
            OUTPUT_RELAY2 = intSwitchStatus;
            
        }
        break;
        case '3':
        {
            OUTPUT_RELAY3 = intSwitchStatus;
        }
        break;
        case '4':
        {
            OUTPUT_RELAY4 = intSwitchStatus;
        }
        break;
        default:
            break;
    }
}