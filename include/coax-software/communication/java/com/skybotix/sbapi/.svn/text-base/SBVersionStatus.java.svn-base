package com.skybotix.sbapi;
import java.io.OutputStream;

import com.skybotix.sbapi.SBSerialisedMessage;

/* Type representation of the version information, maps directly to the message
 * type */
public class SBVersionStatus {
	/**
	 * Request the version information from the target system:
	 * apiVersion is the version of the communication API. It should be the same on
	 * both end of the communication link
	 * controllerVersion is the version of the controller running on the target
	 * platform. 
	 * compileTime is the textual representation of the compile time, obtained from
	 * the preprocessor macro __DATE__
	 * */
	public short apiVersion;
	public short controllerVersion;
	public byte[] imuVersion = new byte[SBConsts.SB_IMU_VERSION_LENGTH];
	public byte[] compileTime = new byte[SBConsts.SB_COMPILE_TIME_LENGTH];

	/** 
	 * These will be used to check that the compile options in the library
	 * are the same as in the linking program
	 **/
	public short sizeOf_SBApiSimpleContext;
	public short sizeOf_SBControlContext;
	public short sizeOf_SBHeliState;
	public short sizeOf_SBHeliStateRaw;

    void sbPrintVersion(OutputStream out) {
    }

    int encode(SBSerialisedMessage msg, byte handle) {
        int i,o=SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
        byte[] m = msg.data;
        msg = new SBSerialisedMessage(SBConsts.SB_MSGID_GET_VERSION | SBConsts.SB_MSGID_REPLY, handle);
        m[o+0] = (byte)(apiVersion);
        m[o+1] = (byte)(apiVersion >>> 8);
        m[o+2] = (byte)(controllerVersion);
        m[o+3] = (byte)(controllerVersion >>> 8);
        o += 4;
        for (i=0;i<SBConsts.SB_COMPILE_TIME_LENGTH;i++) {
            m[o+i] = compileTime[i];
        }
        o += SBConsts.SB_COMPILE_TIME_LENGTH;
        for (i=0;i<SBConsts.SB_IMU_VERSION_LENGTH;i++) {
            m[o+i] = imuVersion[i];
        }
        msg.len = SBConsts.SB_COMPILE_TIME_LENGTH + SBConsts.SB_IMU_VERSION_LENGTH + 4;
        return msg.sbFinalizeMessage();
    }

    int decode(SBSerialisedMessage msg) {
        int i,o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
        byte[] m = msg.data;
        if (msg.sbCheckDecode("sbVersionMsgDecode",SBConsts.SB_MSGID_GET_VERSION,
                    SBConsts.SB_COMPILE_TIME_LENGTH+SBConsts.SB_IMU_VERSION_LENGTH+4) != 0) {
            return -1;
        }
        apiVersion = msg.shortofch2(m[o+0],m[o+1]);
        controllerVersion = msg.shortofch2(m[o+2],m[o+3]);
        o += 4;
        for (i=0;i<SBConsts.SB_COMPILE_TIME_LENGTH;i++) {
            compileTime[i] = m[o+i];
        }
        o += SBConsts.SB_COMPILE_TIME_LENGTH;
        for (i=0;i<SBConsts.SB_IMU_VERSION_LENGTH;i++) {
            imuVersion[i] = m[o+i];
        }
        return 0;
    }
} 

