using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
 
public class TcpConnection : Connection
{
    public TcpClient tcpClient;
    public NetworkStream stream;
 
    public override void StartConnection(string sendIp, int sendPort, int receivePort)
    {
        //IPEndPoint ipEndPoint = new IPEndPoint(IPAddress.Parse(sendIp), senderPort);
        tcpClient = null;

        if (sendIp.Length <= 15)
        {
            try
            {
                tcpClient = new TcpClient(AddressFamily.InterNetwork);
            } //InterNetworkV6
            catch (Exception e)
            {
                Debug.Log("Failed to listen for UDP at port " + receivePort + ": " + e.Message);
                return;
            }
        }
        else
        {
            try
            {
                tcpClient = new TcpClient(AddressFamily.InterNetworkV6);
                tcpClient.Client.DualMode = true;
            } //InterNetworkV6
            catch (Exception e)
            {
                Debug.Log("Failed to listen for UDP at port " + receivePort + ": " + e.Message);
                return;
            }
        }
        try { 
            tcpClient.Connect(sendIp, port: sendPort);
            tcpClient.ReceiveBufferSize = 1024000;
            stream = tcpClient.GetStream();
         } //InterNetworkV6
        catch (Exception e)
        {
            Debug.Log("Failed to listen for TCP at port " + receivePort + ": " + e.Message);
            return;
        }
        this.senderIp = sendIp;
        this.senderPort = sendPort;
  
        //await client.ConnectAsync(ipEndPoint);
        //int received = await stream.ReadAsync(buffer);


        Debug.Log("Set sendee at ip " + sendIp + " and port " + sendPort);
 
        
    }
 
    public override void StartReceiveThread()
    {
        receiveThread = new Thread(() => ListenForMessages(this));
        receiveThread.IsBackground = true;
        threadRunning = true;
        receiveThread.Start();
    }
    private void clearReadBuffer(TcpConnection tcpConnection)
    {
        try
        {
            var buffer = new byte[4096];
            while (tcpConnection.stream.DataAvailable)
            {
                tcpConnection.stream.Read(buffer, 0, buffer.Length);
            }
        }
        catch (Exception e)
        {
            Debug.Log("Error clearing the buffer: " + e.Message);
        }
    }
 
    protected void ListenForMessages(TcpConnection tcpConnection)
    {
        int bytesReaded, bytesToRead, len, numBytesToRead;
        int bufferPos;
        var typeSizeBuffer = new byte[5]; //buffer just to store the type and size

        while (threadRunning)
        {
            if(tcpConnection.tcpClient == null || !tcpConnection.tcpClient.Connected)
            {
                //IPEndPoint ipEndPoint = new IPEndPoint(IPAddress.Parse(this.senderIp), this.senderPort);
                try { 
                    tcpConnection.tcpClient = new TcpClient(AddressFamily.InterNetworkV6);
                    tcpConnection.tcpClient.Client.DualMode = true;
                    tcpConnection.tcpClient.ReceiveBufferSize = 1024000;
                    tcpConnection.tcpClient.Connect(tcpConnection.senderIp, port:  tcpConnection.senderPort);
                    tcpConnection.stream = tcpConnection.tcpClient.GetStream();
                } //InterNetworkV6
                catch (Exception e)
                {
                    Debug.Log("Failed to connect at port " + this.senderPort + ": " + e.Message);
                    Thread.Sleep(1000);
                    continue;
                }
            }
            //nothing to receive?
            if(tcpConnection.tcpClient.Available == 0)
            {
                Thread.Sleep(1);
                continue;
            }

            try
            {
                bufferPos = 0;
                //reads 1 byte from the stream and up it at pos 0 of the buffer
                bytesReaded = tcpConnection.stream.Read(typeSizeBuffer, bufferPos, 1);
                bufferPos++;
                
                try
                {
                    Byte[] receiveBytes;
                    switch((int)typeSizeBuffer[0])
                    {
                        case (int)PacketType.SOUND:
                            //the next 4 bytes are the image length
                            bytesReaded = tcpConnection.stream.Read(typeSizeBuffer, bufferPos, 4);
                            bufferPos += 4;
                            if(bytesReaded < 4)
                            {
                                Debug.Log("Readed different than 4 bytes for a sound len. Discarting message.");
                                clearReadBuffer(tcpConnection);
                                continue;
                            }

                            
                            len = typeSizeBuffer[1];// = (picture->len & 0xFF);
                            len += typeSizeBuffer[2] << 8;
                            len += typeSizeBuffer[3] << 16;
                            len += typeSizeBuffer[4] << 24;

                            //The sound len must be 4000
                            if(len != 4000)
                            {
                                Debug.Log("Readed sound len is different than 4000 bytes for a sound len. Discarting message.");
                                clearReadBuffer(tcpConnection);   
                                continue;
                            }
                            receiveBytes = new byte[5+len]; // Buffer to queue
          
                            //play the sound we just received
                            //the first byte is te type
                            //reads 4000 bytes from the stream and up it at pos 1 of the buffer
                            bytesReaded = 0;
                            bytesToRead = 4000;
                            while(bytesReaded != bytesToRead)
                            {
                                numBytesToRead = tcpConnection.tcpClient.Available;
                                if(numBytesToRead > bytesToRead-bytesReaded)
                                    numBytesToRead = (int)bytesToRead-bytesReaded;
                                numBytesToRead = tcpConnection.stream.Read(receiveBytes, bufferPos, numBytesToRead);
                                bytesReaded += numBytesToRead;
                                bufferPos += numBytesToRead;
                            }
                            //if(bytesRead != 4000)
                            //{
                            //    Debug.Log("Readed different than 4000 bytes for a sound packet. Discarting message.");
                            //    continue;
                            //}
                            //copy the type and size
                            receiveBytes[0] = typeSizeBuffer[0];
                            receiveBytes[1] = typeSizeBuffer[1];
                            receiveBytes[2] = typeSizeBuffer[2];
                            receiveBytes[3] = typeSizeBuffer[3];
                            receiveBytes[4] = typeSizeBuffer[4];

                            //add the message to the queue
                            lock (incomingQueue)
                            {
                                incomingQueue.Enqueue(receiveBytes);
                            }

                        break;
                        case (int)PacketType.IMAGE:
                            //the next 4 bytes are the image length
                            bytesReaded = tcpConnection.stream.Read(typeSizeBuffer, bufferPos, 4);
                            bufferPos += 4;
                            if(bytesReaded < 4)
                            {
                                Debug.Log("Readed different than 4 bytes for a image len. Discarting message.");
                                clearReadBuffer(tcpConnection);   
                                continue;
                            }

                            
                            len = typeSizeBuffer[1];// = (picture->len & 0xFF);
                            len += typeSizeBuffer[2] << 8;
                            len += typeSizeBuffer[3] << 16;
                            len += typeSizeBuffer[4] << 24;

                            receiveBytes = new byte[5+len]; // Buffer to queue
                            if(len > 100000 || len < 6000)
                            {
                                Debug.Log("Invalid Len. Discarting message.");
                                clearReadBuffer(tcpConnection);   
                                continue;
                            }


                            //play the sound we just received
                            //the first byte is te type
                            //reads 4000 bytes from the stream and up it at pos 1 of the buffer
                            bytesReaded = 0;
                            bytesToRead = len;
                            while(bytesReaded != bytesToRead)
                            {
                                numBytesToRead = tcpConnection.tcpClient.Available;
                                if(numBytesToRead > bytesToRead-bytesReaded)
                                    numBytesToRead = (int)bytesToRead-bytesReaded;
                                numBytesToRead = tcpConnection.stream.Read(receiveBytes, bufferPos, numBytesToRead);
                                bytesReaded += numBytesToRead;
                                bufferPos += numBytesToRead;
                            }

                            //if(bytesReaded < len)
                            //{
                            //    Debug.Log("Readed different than " + len + " bytes for a image packet. Discarting message.");
                            //    continue;
                            //}

                            //copy the type and size
                            receiveBytes[0] = typeSizeBuffer[0];
                            receiveBytes[1] = typeSizeBuffer[1];
                            receiveBytes[2] = typeSizeBuffer[2];
                            receiveBytes[3] = typeSizeBuffer[3];
                            receiveBytes[4] = typeSizeBuffer[4];

                            //add the message to the queue
                            lock (incomingQueue)
                            {
                                incomingQueue.Enqueue(receiveBytes);
                            }
                        break;
                        default:
                            Debug.Log("Unknown message. Discarting data.");
                            clearReadBuffer(tcpConnection);   
                            continue;
                        break;

                    }
                }
                catch (Exception e)
                {
                    Debug.Log("Error update data from client: " + e.Message);
                }
            }
            catch (SocketException e)
            {
                // 10004 thrown when socket is closed
                if (e.ErrorCode != 10004) Debug.Log("Socket exception while receiving data from tcp client: " + e.Message);
            }
            catch (Exception e)
            {
                Debug.Log("Error receiving data from tcp client: " + e.Message);
            }
            Thread.Sleep(1);
        }
    }
 /*
    public string[] getMessages()
    {
        string[] pendingMessages = new string[0];
        lock (incomingQueue)
        {
            pendingMessages = new string[incomingQueue.Count];
            int i = 0;
            while (incomingQueue.Count != 0)
            {
                pendingMessages[i] = incomingQueue.Dequeue();
                i++;
            }
        }
 
        return pendingMessages;
    }*/
 
    public override void Send(string message)
    {
        Debug.Log(String.Format("Send msg to ip:{0} port:{1} msg:{2}",senderIp,senderPort,message));
        //IPEndPoint serverEndpoint = new IPEndPoint(IPAddress.Parse(senderIp), senderPort);
        Byte[] sendBytes = Encoding.UTF8.GetBytes(message);
        //udpClient.Send(sendBytes, sendBytes.Length, serverEndpoint);
        stream.Write(sendBytes,0, sendBytes.Length);
    }
    public override void Send(Byte[] sendBytes)
    {
        if(senderIp != null)
        {
            //IPEndPoint serverEndpoint = new IPEndPoint(IPAddress.Parse(senderIp), senderPort);
            //udpClient.Send(sendBytes, sendBytes.Length, serverEndpoint);
            stream.Write(sendBytes,0, sendBytes.Length);
        }
    }
    public override void Send(Byte[] sendBytes, int length)
    {
        if(senderIp != null)
        {
            //IPEndPoint serverEndpoint = new IPEndPoint(IPAddress.Parse(senderIp), senderPort);
            //udpClient.Send(sendBytes, length, serverEndpoint);
            stream.Write(sendBytes,0, length);
        } 
    }
 
    public override void Stop()
    {
        threadRunning = false;
        receiveThread.Abort();
        tcpClient.Close();
    }
}
 