using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
 
public abstract  class Connection
{ 
    public enum PacketType {
        CONNECTING,
        DISCONNECTING,
        CHANGE_12_MOTORS_STEPS,
        SOUND_SHORT,//SOUND_AND_CHANGE_12_MOTORS_STEPS,
        SOUND, 
        IMAGE,
        COMMAND //TYPE + 9 bytes of command. The first byte is sound active
    };
    protected readonly Queue<Byte[]> incomingQueue = new Queue<Byte[]>();
    protected Thread receiveThread;
    protected bool threadRunning = false;
    protected string senderIp;
    protected int senderPort;
 
    public Byte[] getMessage()
    {
        Byte[] recBytes = new Byte[0];
        lock (incomingQueue)
        {
            if(incomingQueue.Count != 0)
            {
                recBytes = incomingQueue.Dequeue();
            } else {

                return null;
            }
        }
        return recBytes;
    }

    public abstract  void StartConnection(string sendIp, int sendPort, int receivePort);
 
    public abstract void StartReceiveThread();
 
  
    public abstract void Send(string message);

    public abstract void Send(Byte[] sendBytes);

    public abstract void Send(Byte[] sendBytes, int length);
 
    public abstract void Stop();
}
 