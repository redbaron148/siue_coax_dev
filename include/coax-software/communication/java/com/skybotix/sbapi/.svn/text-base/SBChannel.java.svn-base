package com.skybotix.sbapi;

public interface SBChannel {
    final public int SB_CHANNEL_UDP = 1;
    final public int SB_CHANNEL_BTANDROID = 2;

    public int channelType = 0;

    int open();
    int close();

    /*   These functions can be used to protect the channel when SBC_HAS_PTHREAD  */
    /*   is set. They will fail returning -1 otherwise.  */
    int lock();
    int unlock();

    int send( byte[] data, int offset, int size);
    int receive( byte[] data, int offset, int size);

    int sendAll( byte[] data, int offset, int size,int timeout_ms);
    int waitBuffer( byte[] data, int offset, int size,int timeout_ms);
    int waitData( int timeout_ms);

    int flush();

    int sendString(String string, int timeout_ms);

}



