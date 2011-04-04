package com.skybotix.sbapi;
import java.io.IOException;
import java.io.OutputStream;
import java.io.InputStream;
import com.skybotix.sbapi.SBChannel;
import com.skybotix.sbapi.SBConsts;
import com.skybotix.sbapi.CRCTable;

public class SBSerialisedMessage {
    final static public byte SB_MESSAGE_HEADER_LEN = 4;
    final static public byte SB_MAX_MESSAGE_SIZE = 110;
    final static public byte SB_MESSAGE_START_FRAME = (byte)0xCD;
    final static private CRCTable crcTable = new CRCTable();


    public byte msgid = 0;
    public byte handle = 0;
    public byte len = 0;
    // bytes 3-105 will be usable to store message body data
    public byte[] data = new byte[SB_MAX_MESSAGE_SIZE]; 
    public byte crc = 0;

    public SBSerialisedMessage(int msgid, int handle) {
        data = new byte[SB_MAX_MESSAGE_SIZE];
        msgid = (byte)msgid;
        handle = (byte)handle;
        len = 0;
    }

    public static short shortofch2(byte i0,byte i8) {
        return (short)((short)i0 | ((short)i8 << 8));
    }
    public static long longofch4(byte a,byte b,byte c,byte d) {
        return ((long)a | ((long)b << 8) | ((long)c << 16) | ((long)d << 24));
    }

    byte computeCRC() {
        return crcTable.compute(data,len+SB_MESSAGE_HEADER_LEN);
    }

    public int sbFinalizeMessage() {
        data[0] = SB_MESSAGE_START_FRAME;
        data[1] = msgid;
        data[2] = handle;
        data[3] = len;
        data[len+SB_MESSAGE_HEADER_LEN] = crc = computeCRC();
        return 0;
    }

    public int sbValidateMessage() {
        byte _crc = computeCRC();
        if (len > SB_MAX_MESSAGE_SIZE-SB_MESSAGE_HEADER_LEN) {
            return -3;
        }

        if (len+SB_MESSAGE_HEADER_LEN >= SB_MAX_MESSAGE_SIZE) {
            System.out.printf("sbValidateMessage: invalid size\n");
            return -2;
        }
        if (_crc != crc) {
            sbDumpMessage(System.out,"Validation failed:");
            System.out.printf("sbValidateMessage(%02X): invalid CRC (got %02X expected %02X)\n",msgid,crc,crc);
            return -1;
        }
        return 0;
    }

    public int sbCheckDecode(String name, int mid, int mlen) {
        if ((msgid & ~(SBConsts.SB_MSGID_REQACK|SBConsts.SB_MSGID_REPLY)) != mid) {           
            System.out.printf("%s: invalid msgid: %02X/%02X instead of %02X\n",name, 
                    msgid,msgid & ~(SBConsts.SB_MSGID_REQACK|SBConsts.SB_MSGID_REPLY),mid);      
            return -1;                                     
        }                                                  
        if (len != mlen) {                             
            System.out.printf("%s: invalid msg len: %d instead of %d\n",name,len,mlen);    
            return -1;                                     
        }
        return 0;
    }

    public void sbDumpMessage(OutputStream out, String prefix) {
        int i;
        String sout = String.format("%s Message: id %02X handle %d len %d [",prefix,msgid,handle,len);
        for (i=0;i<len;i++) {
            sout += String.format("%02X ",data[i+SB_MESSAGE_HEADER_LEN]);
        }
        sout += String.format("] CRC %02X\n",crc);
        try {
            out.write(sout.getBytes());
        } catch (IOException e) {
            System.out.print(sout);
        }
    }

    public int sbWaitRawMessage(SBChannel channel, int expmsgid, int timeout_ms) {
        int n = 0;
        int numtries = 10;
        while (numtries>0) {
            numtries --;
            while (true) { 
                if (channel.waitBuffer(data, 0, 1, timeout_ms) != 0) {
                    System.out.printf("Ouch\n");
                    return -1;
                }
                if (data[0] == SB_MESSAGE_START_FRAME) {
                    break;
                }
                System.out.printf("Discarding %02X\n",data[0]);
            }
            if (channel.waitBuffer(data, 1, SB_MESSAGE_HEADER_LEN-1, timeout_ms) != 0) {
                System.out.printf("Arghhh\n");
                return -1;
            } else {
                // printf(".");fflush(stdout);
                msgid = data[1];
                handle = data[2];
                len = data[3];
                n = len + 1;
                // printf("Packet %02X H%d len %d\n",msgid,handle,len);
            }
            if (len > SB_MAX_MESSAGE_SIZE-SB_MESSAGE_HEADER_LEN) {
                System.out.printf("Rotten packet\n");
                return -5;
            }
            if (channel.waitBuffer(data, SB_MESSAGE_HEADER_LEN, n, timeout_ms) != 0) {
                System.out.printf("Crack\n");
                return -2;
            } else {
                // printf("!");fflush(stdout);
                crc = data[len+SB_MESSAGE_HEADER_LEN];
            }
            if (sbValidateMessage() != 0) { 
                System.out.printf("Pop\n");
                return -3; 
            }
            if ((expmsgid>=0) && (expmsgid != msgid)) {
                System.out.printf("Message with incorrect id (%d instead of %d), continuing\n",
                        msgid, expmsgid);
                continue;
            }
            return 0;
        }
        return -1;
    }

    public int sbSendMessage(SBChannel channel, int timeout_ms) {
        return channel.sendAll(data,0,len+SB_MESSAGE_HEADER_LEN+1,timeout_ms);
    }

    public int sbSaveMessage(OutputStream out) throws IOException {
        out.write(data,0,len+4);
        return 0;
    }

    public int sbLoadMessage(InputStream in) throws IOException {
        int crcRead,crcComputed,n;
        if (in.read(data,0,3)!=3) {
            System.out.printf("loadMessage: failed to read message header\n");
            return -1;
        }
        msgid = data[0];
        handle = data[1];
        len = data[2];
        n = len + 1;
        if (in.read(data,3,n) != n) {
            System.out.printf("loadMessage: failed to read message body\n");
            return -2;
        }
        crcRead = data[len+3];
        crcComputed = computeCRC();
        if (crcRead != crcComputed) {
            System.out.printf("loadMessage: invalid CRC\n");
            return -3;
        }
        return 0;
    }
} 
