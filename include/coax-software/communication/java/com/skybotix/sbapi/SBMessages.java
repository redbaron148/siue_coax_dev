package com.skybotix.sbapi;
import java.io.IOException;
import java.io.OutputStream;
import java.io.InputStream;
import com.skybotix.sbapi.SBChannel;
import com.skybotix.sbapi.SBConsts;
import com.skybotix.sbapi.SBSerialisedMessage;
import com.skybotix.sbapi.SBControlContext;

public class SBMessages {

    public class SBStringMessage {
        byte[] text = new byte[SBConsts.SB_STRING_MESSAGE_LENGTH];

        int encode(SBSerialisedMessage msg, int handle) {
            int i;
            msg = new SBSerialisedMessage(SBConsts.SB_MSGID_STRING, handle);
            for (i=0;i<SBConsts.SB_STRING_MESSAGE_LENGTH;i++) {
                msg.data[SBSerialisedMessage.SB_MESSAGE_HEADER_LEN+i] = text[i];
            }
            msg.len = SBConsts.SB_STRING_MESSAGE_LENGTH;
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int i;
            if (msg.sbCheckDecode("sbStringMsgDecode",SBConsts.SB_MSGID_STRING,
                        SBConsts.SB_STRING_MESSAGE_LENGTH) != 0) {
                return -1;
            }
            for (i=0;i<SBConsts.SB_STRING_MESSAGE_LENGTH;i++) {
                text[i] = msg.data[i+SBSerialisedMessage.SB_MESSAGE_HEADER_LEN];
            }
            return 0;
        }

    }

    public class SBCustomMessage {
        byte[] text = new byte[SBConsts.SB_STRING_MESSAGE_LENGTH];

        int encode(SBSerialisedMessage msg, int handle, 
                int reply) {
            int i;
            if (reply != 0) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CUSTOM|SBConsts.SB_MSGID_REPLY, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CUSTOM, handle);
            }
            for (i=0;i<SBConsts.SB_STRING_MESSAGE_LENGTH;i++) {
                msg.data[SBSerialisedMessage.SB_MESSAGE_HEADER_LEN+i] = text[i];
            }
            msg.len = SBConsts.SB_STRING_MESSAGE_LENGTH;
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int i;
            if (msg.sbCheckDecode("sbStringMsgDecode",SBConsts.SB_MSGID_CUSTOM,
                        SBConsts.SB_STRING_MESSAGE_LENGTH) != 0) {
                return -1;
            }
            for (i=0;i<SBConsts.SB_STRING_MESSAGE_LENGTH;i++) {
                text[i] = msg.data[i+SBSerialisedMessage.SB_MESSAGE_HEADER_LEN];
            }
            return 0;
        }

    }



    public class SBConfigureCommLoop {
        byte verbosity;
        byte debug_channel;
        int encode(SBSerialisedMessage msg, int handle,  boolean requestAck) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (requestAck) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP | SBConsts.SB_MSGID_REQACK, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            }
            m[o+0] = verbosity;
            m[o+1] = debug_channel;
            msg.len = 2;
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            verbosity = m[o+0];
            debug_channel = m[o+1];
            return 0;
        }

    }

    public class SBConfigureObstAvoid {
        byte oavoidMode;
        int encode(SBSerialisedMessage msg, int handle,  boolean requestAck) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (requestAck) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_OAVOID | SBConsts.SB_MSGID_REQACK, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_OAVOID, handle);
            }
            m[o+0] = oavoidMode;
            msg.len = 1;
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureObstAvoid",SBConsts.SB_MSGID_CFG_OAVOID, 1) != 0) {
                return -1;
            }
            oavoidMode = m[o+0];
            return 0;
        }

        void print(OutputStream out) throws IOException {
            String sout = String.format("Obst. Avoid: Vertical %d Horizontal %d\n",
                    ((oavoidMode & SBConsts.SB_OA_VERTICAL)!=0)?1:0,
                    ((oavoidMode & SBConsts.SB_OA_HORIZONTAL)!=0)?1:0);
            out.write(sout.getBytes());
        }
    }

    public class SBCoaxSpeedSetLight {
        byte percent;
        int encode(SBSerialisedMessage msg, int handle,  boolean requestAck) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (requestAck) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_SET_LIGHT | SBConsts.SB_MSGID_REQACK, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_SET_LIGHT, handle);
            }
            m[o+0] = percent;
            msg.len = 1;
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBCoaxSpeedSetLight",SBConsts.SB_MSGID_SET_LIGHT, 1) != 0) {
                return -1;
            }
            percent = m[o+0];
            return 0;
        }

    }

    public class SBConfigureCommunication {
        byte commMode;
        byte frequency;
        int numMessages;
        int[] content = new int[2];
        int encode(SBSerialisedMessage msg, int handle,  boolean requestAck) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (requestAck) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMM | SBConsts.SB_MSGID_REQACK, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMM, handle);
            }
            m[o+0] = (byte)(commMode);
            m[o+1] = (byte)(frequency);
            m[o+2] = (byte)(content[1]);
            m[o+3] = (byte)((content[1] >>> 8));
            m[o+4] = (byte)(content[0]);
            m[o+5] = (byte)(numMessages);
            m[o+6] = (byte)(numMessages >>> 8);
            msg.len = 7;
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommunication",SBConsts.SB_MSGID_CFG_COMM, 7) != 0) {
                return -1;
            }
            commMode = m[o+0];
            frequency = m[o+1];
            content[1] = msg.shortofch2(m[o+2],m[o+3]);
            content[0] = msg.shortofch2(m[o+4],(byte)0);
            numMessages = msg.shortofch2(m[o+5],m[o+6]);
            return 0;
        }

        void print(OutputStream out) throws IOException{
            String sout = String.format("Continuous communication: %d\n",commMode);
            out.write(sout.getBytes());
            if (commMode!=0) {
                long lcontent = ((long)content[0]) | (((long)content[1]) << 16);
                sout = String.format("\tFrequency %d Content %04X%04X\n\t",frequency,content[0],content[1]);
                out.write(sout.getBytes());
                SBControlContext.sbContentPrint(out,lcontent);
                out.write("\n".getBytes());
            }
        }
    }


    public class SBConfigureControl {
        byte roll;
        byte pitch;
        byte yaw;
        byte altitude;
        int encode(SBSerialisedMessage msg, int handle,  boolean requestAck) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (requestAck) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_CONTROL | SBConsts.SB_MSGID_REQACK, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_CONTROL, handle);
            }
            m[o+0] = (byte)(((roll & 0x0F) << 0) | ((pitch & 0x0F) << 4));
            m[o+1] = (byte)(((yaw & 0x0F) << 0) | ((altitude & 0x0F) << 4));
            msg.len = 2;
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureControl",SBConsts.SB_MSGID_CFG_CONTROL, 2) != 0) {
                return -1;
            }
            roll = (byte)(m[o+0] & 0x0F);
            pitch = (byte)((m[o+0] >> 4) & 0x0F);
            yaw = (byte)(m[o+1] & 0x0F);
            altitude = (byte)((m[o+1] >> 4) & 0x0F);
            return 0;
        }

        void print(OutputStream out) throws IOException {
            String sout = String.format("Control Modes | Roll %s | Pitch %s | Yaw %s | Alt. %s |\n",
                    SBControlContext.sbCtrlModeString(roll), SBControlContext.sbCtrlModeString(pitch), 
                    SBControlContext.sbCtrlModeString(yaw), SBControlContext.sbCtrlModeString(altitude));
            out.write(sout.getBytes());
        }
    }

    public class SBConfigureTimeout {
        int controlTimeout;
        int watchdogTimeout;
        int encode(SBSerialisedMessage msg, int handle,  boolean requestAck) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (requestAck) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP | SBConsts.SB_MSGID_REQACK, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            }
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }

        void print(OutputStream out) {
        }
    }

    public class SBRequestState {
        int[] content = new int[2];
        int encode(SBSerialisedMessage msg, int handle) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            msg = new SBSerialisedMessage(SBConsts.SB_MSGID_STATE|SBConsts.SB_MSGID_REQACK, handle);
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_STATE, 3) != 0) {
                return -1;
            }
            return 0;
        }

        void print(OutputStream out) {
        }
    }

    public class SBSetNavigationMode {
        byte mode;
        int encode(SBSerialisedMessage msg, int handle,  boolean requestAck) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (requestAck) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP | SBConsts.SB_MSGID_REQACK, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            }
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }

        void print(OutputStream out) {
        }
    }

    public class SBSetControl {
        int roll;
        int pitch;
        int yaw;
        int altitude;
        int encode(SBSerialisedMessage msg, int handle,  boolean requestAck) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (requestAck) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP | SBConsts.SB_MSGID_REQACK, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            }
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }

        void print(OutputStream out) {
        }
    }

    public class SBSetControlWithTimestamp {
        int roll;
        int pitch;
        int yaw;
        int altitude;
        long timestamp;
        int encode(SBSerialisedMessage msg, int handle,  boolean requestAck) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (requestAck) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP | SBConsts.SB_MSGID_REQACK, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            }
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }

    }

    public class SBRawControl {
        int motor1;
        int motor2;
        int servo1;
        int servo2;
        int encode(SBSerialisedMessage msg, int handle,  boolean requestAck) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (requestAck) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP | SBConsts.SB_MSGID_REQACK, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            }
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }

    }

    public class SBConfigureRawControl {
        byte speedprofile1;
        byte speedprofile2;
        int encode(SBSerialisedMessage msg, int handle,  boolean requestAck) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (requestAck) {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP | SBConsts.SB_MSGID_REQACK, handle);
            } else {
                msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            }
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }

        void print(OutputStream out) {
        }
    }

    public class SBTrimModeMessage {
        byte trimMode; // defined in sbconst.h
        // All position multiplied by 10000
        int rollTrim;
        int pitchTrim;

        int encode(SBSerialisedMessage msg, int handle,  int setRequest) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }


        /** Provide a string representation of constant SB_TRIM_xxxx */
        String sbTrimModeString(byte mode) {
            return null;
        }

        void print(OutputStream out) {
        }
    }


    public class SBControlParametersMessage {
        // All gains multiplied by 1000
        int baseThrust; // kh
        int yawOffset; 
        int altitudeKp;
        int altitudeKi;
        int altitudeKd;
        int yawKp;
        int yawKi;
        int yawKd;
        int encode(SBSerialisedMessage msg, int handle,  int setRequest) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }

        void print(OutputStream out) {
        }
    }

    public class SB6DofPoseMessage {
        // All position in mm
        long x,y,z;
        // All angles scaled so that Pi = 0x8000
        int roll;
        int pitch;
        int yaw;
        int encode(SBSerialisedMessage msg, int handle,  int reply) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }

    }

    public class SBConfigureBluetoothMessage {
        byte[] code = new byte[4];
        byte[] name = new byte[16];
        int encode(SBSerialisedMessage msg, int handle,  int setRequest) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }

    }


    // REPLY MESSAGES:
    //


    public class SBBasicReplyMessage {
        byte status; // See SB_REPLY_...
        int encode(SBSerialisedMessage msg, int handle, byte msgid) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }

        void print(OutputStream out) {
        }
    }

    public class SBSensorListMessage {
        int[] content = new int[2]; // same semantic has in RequestState
        int encode(SBSerialisedMessage msg, int handle) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            msg = new SBSerialisedMessage(SBConsts.SB_MSGID_CFG_COMMLOOP, handle);
            return msg.sbFinalizeMessage();
        }

        int decode(SBSerialisedMessage msg) {
            int o = SBSerialisedMessage.SB_MESSAGE_HEADER_LEN;
            byte[] m = msg.data;
            if (msg.sbCheckDecode("SBConfigureCommLoop",SBConsts.SB_MSGID_CFG_COMMLOOP, 2) != 0) {
                return -1;
            }
            return 0;
        }

        void print(OutputStream out) {
        }
    }

}

