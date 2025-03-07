using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEngine.Events;
using UnityEngine.XR;
using System;
using TMPro;

public class ConnectionLogic : MonoBehaviour
{
    


    private Connection connection;
    private InverseKinematic invKin;
    //
    public Boolean tcpConnection = true;
    public float timesPerSecond = 6.0f; 
    public string sendIp;//"127.0.0.1"; //'fe80:0000:0000:0000:9e9c:1fff:fec2:b374'
                            
    private byte[] udpSendBytes = new byte[4400];//1+4000+24];//[25];
        //{
        //    0x02, 0x00, 0x00, 0x00, 0x00, 0x20, 0x4E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        //};
    public Material cameraMaterial; 
    public int sendPort = 3389;
    private float nextTime = 0;
    private float lastStatusChangeTime = 0;
    private Texture2D tex;
    private Vector3 leftHandAngle, rightHandAngle;
    private float rightLeg, leftLeg;
    float angleUpDown;
    public TMP_InputField ipInput;
    public GameObject planes;
    private bool sleeping = true;
    //Vector3 rightArmAdjustment = new Vector3(-0.1728693f, 0.1984623f, 0.1896212f); //-(0.1838693, -0.1744623, -0.1246212)
    //Vector3 rightArmAdjustment = new Vector3(-0.3728693f, 0.1984623f, 0.1896212f); //-(0.1838693, -0.1744623, -0.1246212)
    Vector3 rightArmAdjustment = new Vector3(0.0f, 0.1984623f, 0.0f); //x and z calculated at calibration
    //Vector3 leftArmAdjustment = new Vector3(0.1728693f, 0.1984623f, 0.1896212f); //-(-0.1838693, -0.1744623, -0.1246212)
    Vector3 leftArmAdjustment = new Vector3(0.0f, 0.1984623f, 0.0f); //x and z calculated at calibration

    // Start is called before the first frame update
    void Start()
    {
        //int receivePort = 11000;
       
        // Create a texture. Texture size does not matter, since
        // LoadImage will replace with with incoming image size.
        tex = new Texture2D(2, 2);

        /*
               // A small 64x64 Unity logo encoded into a PNG.
               byte[] pngBytes = new byte[]
               {
                   0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52,
                   0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 0x08, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x02, 0x2E,
                   0x02, 0x00, 0x00, 0x01, 0x57, 0x49, 0x44, 0x41, 0x54, 0x78, 0x01, 0xA5, 0x57, 0xD1, 0xAD, 0xC4,
                   0x30, 0x08, 0x83, 0x81, 0x32, 0x4A, 0x66, 0xC9, 0x36, 0x99, 0x85, 0x45, 0xBC, 0x4E, 0x74, 0xBD,
                   0x8F, 0x9E, 0x5B, 0xD4, 0xE8, 0xF1, 0x6A, 0x7F, 0xDD, 0x29, 0xB2, 0x55, 0x0C, 0x24, 0x60, 0xEB,
                   0x0D, 0x30, 0xE7, 0xF9, 0xF3, 0x85, 0x40, 0x74, 0x3F, 0xF0, 0x52, 0x00, 0xC3, 0x0F, 0xBC, 0x14,
                   0xC0, 0xF4, 0x0B, 0xF0, 0x3F, 0x01, 0x44, 0xF3, 0x3B, 0x3A, 0x05, 0x8A, 0x41, 0x67, 0x14, 0x05,
                   0x18, 0x74, 0x06, 0x4A, 0x02, 0xBE, 0x47, 0x54, 0x04, 0x86, 0xEF, 0xD1, 0x0A, 0x02, 0xF0, 0x84,
                   0xD9, 0x9D, 0x28, 0x08, 0xDC, 0x9C, 0x1F, 0x48, 0x21, 0xE1, 0x4F, 0x01, 0xDC, 0xC9, 0x07, 0xC2,
                   0x2F, 0x98, 0x49, 0x60, 0xE7, 0x60, 0xC7, 0xCE, 0xD3, 0x9D, 0x00, 0x22, 0x02, 0x07, 0xFA, 0x41,
                   0x8E, 0x27, 0x4F, 0x31, 0x37, 0x02, 0xF9, 0xC3, 0xF1, 0x7C, 0xD2, 0x16, 0x2E, 0xE7, 0xB6, 0xE5,
                   0xB7, 0x9D, 0xA7, 0xBF, 0x50, 0x06, 0x05, 0x4A, 0x7C, 0xD0, 0x3B, 0x4A, 0x2D, 0x2B, 0xF3, 0x97,
                   0x93, 0x35, 0x77, 0x02, 0xB8, 0x3A, 0x9C, 0x30, 0x2F, 0x81, 0x83, 0xD5, 0x6C, 0x55, 0xFE, 0xBA,
                   0x7D, 0x19, 0x5B, 0xDA, 0xAA, 0xFC, 0xCE, 0x0F, 0xE0, 0xBF, 0x53, 0xA0, 0xC0, 0x07, 0x8D, 0xFF,
                   0x82, 0x89, 0xB4, 0x1A, 0x7F, 0xE5, 0xA3, 0x5F, 0x46, 0xAC, 0xC6, 0x0F, 0xBA, 0x96, 0x1C, 0xB1,
                   0x12, 0x7F, 0xE5, 0x33, 0x26, 0xD2, 0x4A, 0xFC, 0x41, 0x07, 0xB3, 0x09, 0x56, 0xE1, 0xE3, 0xA1,
                   0xB8, 0xCE, 0x3C, 0x5A, 0x81, 0xBF, 0xDA, 0x43, 0x73, 0x75, 0xA6, 0x71, 0xDB, 0x7F, 0x0F, 0x29,
                   0x24, 0x82, 0x95, 0x08, 0xAF, 0x21, 0xC9, 0x9E, 0xBD, 0x50, 0xE6, 0x47, 0x12, 0x38, 0xEF, 0x03,
                   0x78, 0x11, 0x2B, 0x61, 0xB4, 0xA5, 0x0B, 0xE8, 0x21, 0xE8, 0x26, 0xEA, 0x69, 0xAC, 0x17, 0x12,
                   0x0F, 0x73, 0x21, 0x29, 0xA5, 0x2C, 0x37, 0x93, 0xDE, 0xCE, 0xFA, 0x85, 0xA2, 0x5F, 0x69, 0xFA,
                   0xA5, 0xAA, 0x5F, 0xEB, 0xFA, 0xC3, 0xA2, 0x3F, 0x6D, 0xFA, 0xE3, 0xAA, 0x3F, 0xEF, 0xFA, 0x80,
                   0xA1, 0x8F, 0x38, 0x04, 0xE2, 0x8B, 0xD7, 0x43, 0x96, 0x3E, 0xE6, 0xE9, 0x83, 0x26, 0xE1, 0xC2,
                   0xA8, 0x2B, 0x0C, 0xDB, 0xC2, 0xB8, 0x2F, 0x2C, 0x1C, 0xC2, 0xCA, 0x23, 0x2D, 0x5D, 0xFA, 0xDA,
                   0xA7, 0x2F, 0x9E, 0xFA, 0xEA, 0xAB, 0x2F, 0xDF, 0xF2, 0xFA, 0xFF, 0x01, 0x1A, 0x18, 0x53, 0x83,
                   0xC1, 0x4E, 0x14, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82,
               };
               // Load data into the texture.
               tex.LoadImage(pngBytes);
               //cameraMaterial
               cameraMaterial.mainTexture = tex;
       */
                invKin = new InverseKinematic();

        /*
               Vector3 v1;
               v1.x = 0.7797F; //in real world units = meters
               v1.y = 0;
               v1.z = 0;
               //Vector3 ang = invKin.obtainAngles(v1, true);

               v1.x = 0;
               v1.y = 0;
               v1.z = 0.7797F; //in real world units = meters
               //ang = invKin.obtainAngles(v1, true);
        */
        if (tcpConnection)
        {
            connection = new TcpConnection();
        }
        else
        {
            SoundScript.SAMPLESCOUNTMIC = 1334; //UP connection limitation
            connection = new UdpConnection();
        }
        ipInput.text = sendIp;

    }
    public void connectToJaeger()
    {
        sendIp = ipInput.text;
        planes.SetActive(true);
        connection.StartConnection(sendIp, sendPort, sendPort);//, receivePort);
        connection.StartReceiveThread();
        //send the first information
        byte[] sendBytes = new byte[]
        {
            0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        };
        connection.Send(sendBytes);
    }
    //The head angle must be obtained only at calibration moment
    float horizontalAngle = 0;
    private bool updateControlersAngles()
    {
        bool updatedAngles = false;
        Vector3 headAngles;
        bool calibrationButton = false;
        bool secButton = false;

        var headDevices = new List<UnityEngine.XR.InputDevice>();
        UnityEngine.XR.InputDevices.GetDevicesAtXRNode(UnityEngine.XR.XRNode.Head, headDevices);
        if(headDevices.Count == 0)
            return false;

        Quaternion headRotation;
        Vector3 headPosition;
        UnityEngine.XR.InputDevice headDevice = headDevices[0];
        if (!headDevice.TryGetFeatureValue(CommonUsages.devicePosition, out headPosition) || !headDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out headRotation))
            return false;
        headAngles = headRotation.eulerAngles;

        //verify if we should change to the sleeping mode/wake up
        var rightHandDevices = new List<UnityEngine.XR.InputDevice>();
        UnityEngine.XR.InputDevices.GetDevicesAtXRNode(UnityEngine.XR.XRNode.RightHand, rightHandDevices);
        if(rightHandDevices.Count == 1)
        {
            UnityEngine.XR.InputDevice device = rightHandDevices[0];
            Vector3 position;
            device.TryGetFeatureValue(CommonUsages.primaryButton, out calibrationButton);
            device.TryGetFeatureValue(CommonUsages.secondaryButton, out secButton);

            //if the button A is pressed
            if (calibrationButton)
            {
                //1 sec min. between changes
                if (Time.time - 1 > lastStatusChangeTime)
                {             
                    lastStatusChangeTime = Time.time;    
                    if(sleeping)
                    {
                        //update the head angle at calibration
                        horizontalAngle = headAngles[1];
                        //calibrate
                        if (device.TryGetFeatureValue(CommonUsages.devicePosition, out position))
                        {
                            Vector3 relativePosition = position - headPosition;
                            //Rotate in order to place locate the hand relative to the headset
                            relativePosition = Quaternion.Euler(0, -headAngles[1], 0) * relativePosition;
                            //update the adjustment
                            rightArmAdjustment.x = -relativePosition.x;
                            rightArmAdjustment.z = -relativePosition.z;
                            //leftArmAdjustment = rightArmAdjustment;
                            //leftArmAdjustment.x = -leftArmAdjustment.x;
                            //leftArmAdjustment.z = -leftArmAdjustment.z;

                            relativePosition += rightArmAdjustment;
                            invKin.calibrate(relativePosition);
                        }

                        //just wake up and keep going
                        sleeping = false;
                    } else {
                        //update the angles to all zeros
                        leftHandAngle.x = 0;
                        rightHandAngle.x = 0;
                        leftHandAngle.y = 0;
                        rightHandAngle.y = 0;
                        leftHandAngle.z = 0;
                        rightHandAngle.z = 0;
                        angleUpDown = 0;
                        invKin.updateCurrentAngles(leftHandAngle, false);
                        invKin.updateCurrentAngles(rightHandAngle, true);
 
                        //and start sleeping sending the zeroed angles
                        sleeping = true;
                        return true;
                    }
                }
            } //if button B is pressed
            else if(secButton)
            {
                //
            }
        }

        if(sleeping)
            return false;

        

        var leftHandDevices = new List<UnityEngine.XR.InputDevice>();
        UnityEngine.XR.InputDevices.GetDevicesAtXRNode(UnityEngine.XR.XRNode.LeftHand, leftHandDevices);
        if(leftHandDevices.Count == 1)
        {
            UnityEngine.XR.InputDevice device = leftHandDevices[0];
            Vector3 position;
            if (device.TryGetFeatureValue(CommonUsages.devicePosition, out position))
            {
                Vector3 relativePosition = position - headPosition;
                //Rotate in order to place locate the hand relative to the headset
                relativePosition = Quaternion.Euler(0, -horizontalAngle, 0) * relativePosition;
                //if we just calibrated
                if (calibrationButton)
                {
                    leftArmAdjustment.x = -relativePosition.x;
                    leftArmAdjustment.z = -relativePosition.z;                        
                }
                
                relativePosition += leftArmAdjustment;
                //relativePosition.x = 0;
                //relativePosition.y = 0.3722f;
                //relativePosition.z = 0.3722f;
                //Debug.Log(string.Format("Device name '{0}' with role '{1}' and position '{2}' and relative position '{3}'", device.name, device.role.ToString(), position, relativePosition));
                
                leftHandAngle = invKin.obtainAngles(relativePosition, false);
                updatedAngles = true;
            }
            Vector2 movVec;
            if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out movVec))
            {
                if(invKin.moveLeg(false, movVec))
                    updatedAngles = true;
            }
            //the right and left legs does not take into account the current state (angle)
            //0 means no movement, while anything > or < will make the legs to move.
            leftLeg = invKin.obtainLeg(false);

            //CommonUsages.triggerButton
        }

        //var rightHandDevices = new List<UnityEngine.XR.InputDevice>();
        //UnityEngine.XR.InputDevices.GetDevicesAtXRNode(UnityEngine.XR.XRNode.RightHand, rightHandDevices);
        if(rightHandDevices.Count == 1)
        {
            UnityEngine.XR.InputDevice device = rightHandDevices[0];
            Vector3 position;
            if (device.TryGetFeatureValue(CommonUsages.devicePosition, out position))
            {
                Vector3 relativePosition = position - headPosition;
                //Rotate in order to place locate the hand relative to the headset
                relativePosition = Quaternion.Euler(0, -horizontalAngle, 0) * relativePosition;
                relativePosition += rightArmAdjustment;
                //relativePosition.x = 0.3722f;
                //relativePosition.y = 0.3722f;
                //relativePosition.z = 0;
                //relativePosition.x = -1*0.3722f;
                //relativePosition.y = 0*0.3722f;
                //relativePosition.z = 1*0.3722f;

                Debug.Log(string.Format("Device name '{0}' with role '{1}' and position '{2}' and relative position '{3}'", device.name, device.role.ToString(), position, relativePosition));
                
                rightHandAngle = invKin.obtainAngles(relativePosition, true);
                //invKin.precalculate(relativePosition, true);
                updatedAngles = true;
                //Debug.Log("p.x: " + relativePosition.x + ", p.y: " + relativePosition.y + ", p.z: " + relativePosition.z +" - r.x: " + rightHandAngle.x + ", r.y: " + rightHandAngle.y + ", r.z: " + rightHandAngle.z + "\n");
            }
            Vector2 movVec;
            if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out movVec))
            {
                if(invKin.moveLeg(true, movVec))
                    updatedAngles = true;
            }
            //the right and left legs does not take into account the current state (angle)
            //0 means no movement, while anything > or < will make the legs to move.
            rightLeg = invKin.obtainLeg(true);
        }            
            
        //we want to monitor the headAngles[0] from (up)270...360...90(down)
        if(headAngles[0] > 180)
        {
            angleUpDown = 360-headAngles[0];//from (up)270 to 360(mid)
            //angleUpDown = -(360-headAngles[0]);//from (up)270 to 360(mid)
            updatedAngles = true;
        }
        else if(headAngles[0] == 0)
        {
            angleUpDown = 0;
            updatedAngles = false;
        }
        else
        {
            angleUpDown = -headAngles[0]; //from 0(mid) to 90(down)
            //angleUpDown = headAngles[0]; //from 0(mid) to 90(down)
            updatedAngles = true;
        }
        if(angleUpDown > 45)
            angleUpDown = 45;
        else if(angleUpDown < -70)
            angleUpDown = -70;
        //Debug.Log(string.Format("Device name '{0}' with role '{1}' and position '{2}' and rotation '{3}'", headDevice.name, headDevice.role.ToString(), headPosition, angleUpDown));

        return updatedAngles;
    }

    private void updateSendBytes(byte[] sendBytes, ref int initialPos)
    {
        //Vector3 centiLeftHand = new Vector3(0,0,0);// = leftHandAngle*100;
        //Vector3 centiRightHand = new Vector3(0,0,0);// = rightHandAngle*100;
        Vector3 centiLeftHand = leftHandAngle*100;
        Vector3 centiRightHand = rightHandAngle*100;
        int centiAngleUpDown = (int)(angleUpDown*100);
        //float centiLeftLeg = leftLeg;
        //float centiRightLeg = rightLeg;
        int val, index, indexVe;

        //sendBytes[initialPos] = 2;
        //Motor Index 0, left leg: 1 & 2
        index = 0;
        val = (int)leftLeg;
        indexVe = initialPos + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        //Motor Index 2, right leg: 3 & 4
        index = 2;
        val = (int)rightLeg;
        indexVe = initialPos + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        ///

        //Motor Index 1, camera
        index = 1;
        val = (int)centiAngleUpDown;
        indexVe = initialPos + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);
        

        ////

        //Motor Index 8, first xrot left hand
        index = 8;
        val = (int)centiLeftHand.x;
        indexVe = initialPos + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        //Motor Index 7, first zrot left hand
        index = 7;
        val = (int)centiLeftHand.y;
        indexVe = initialPos + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        //Motor Index 6, second xrot left hand
        index = 6;
        val = (int)centiLeftHand.z;
        indexVe = initialPos + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        ////

        //Motor Index 5, first xrot right hand
        index = 5;
        val = (int)centiRightHand.x;
        indexVe = initialPos + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        //Motor Index 4, first zrot right hand
        index = 4;
        val = (int)centiRightHand.y;
        indexVe = initialPos + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        //Motor Index 3, second xrot right hand
        index = 3;
        val = (int)centiRightHand.z;
        indexVe = initialPos + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);
        initialPos = initialPos+24;//Motor positions have 1+12*2 bytes initialPos + 8*2 + 2;
/*
        //Motor Index 5, first xrot left hand
        index = 5;
        val = (int)centiLeftHand.x;
        indexVe = 1 + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        //Motor Index 4, first zrot left hand
        index = 4;
        val = (int)centiLeftHand.y;
        indexVe = 1 + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        //Motor Index 3, second xrot left hand
        index = 3;
        val = (int)centiLeftHand.z;
        indexVe = 1 + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        ////

        //Motor Index 8, first xrot right hand
        index = 8;
        val = (int)centiRightHand.x;
        indexVe = 1 + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        //Motor Index 7, first zrot right hand
        index = 7;
        val = (int)centiRightHand.y;
        indexVe = 1 + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);

        //Motor Index 6, second xrot right hand
        index = 6;
        val = (int)centiRightHand.z;
        indexVe = 1 + index*2;
        sendBytes[indexVe] = (byte)(val & 0xFF);
        sendBytes[indexVe+1] = (byte)((val >> 8) & 0xFF);
*/
    }
    // Update is called once per frame
    void Update()
    {
        //bool  gotImage = false;
        if(connection == null)
            return;
        //We check if it is time to send another broadcast
        //
        byte[] recBytes;
        if((recBytes = connection.getMessage()) != null)
        {
            try
            {
                switch((int)recBytes[0])
                {
                    case (int)Connection.PacketType.SOUND:
                        //play the sound we just received
                        //the first byte is te type

                        //if the packet had only the type informations
                        if(recBytes.Length == 1)
                        {
                            //attempt to obtain the sound from the next packet
                            recBytes = connection.getMessage();
                            //in this case, the next packet must have exacly the image size
                            if(recBytes == null || recBytes.Length != 4000)
                            {
                                Debug.Log("Couldnt obtain the sound from the next packet. Size: " + recBytes.Length + " bytes.");  
                                break;
                            }
                            SoundScript.adjustSoundAndPlay(recBytes, recBytes.Length);
                        }
                        //else if the packet has all the information we need
                        else if(recBytes.Length >= 4000+1)
                        {
                            var soundBytes = new byte[4000];
                            Array.Copy(recBytes, 1, soundBytes, 0, 4000);
                            SoundScript.adjustSoundAndPlay(soundBytes, soundBytes.Length);
                        } else {
                            Debug.Log("Couldnt obtain the image from the packet.");  // Size: " + recBytes.Length + " bytes.");  
                            break;
                        }
                    break;
                    case (int)Connection.PacketType.IMAGE:
                        //the next 4 bytes are the sound length
                        
                        int len = recBytes[1];// = (picture->len & 0xFF);
                        len += recBytes[2] << 8;
                        len += recBytes[3] << 16;
                        len += recBytes[4] << 24;
                        //if the packet had only the type and size informations
                        if(recBytes.Length == 5)
                        {
                            //attempt to obtain the image from the next packet
                            recBytes = connection.getMessage();
                            //in this case, the next packet must have exacly the image size
                            if(recBytes == null || recBytes.Length != len)
                            {
                                Debug.Log("Couldnt obtain the image from the next packet.");  // Size: " + recBytes.Length + " bytes.");  
                                break;
                            }
                            //in this case, the packet is exacly the image we need
                            tex.LoadImage(recBytes);
                            cameraMaterial.mainTexture = tex;
                        }
                        //else if the packet has all the information we need
                        else if(recBytes.Length >= len+5)
                        {
                            //Update the image
                            var imageBytes = new byte[len];
                            Array.Copy(recBytes, 5, imageBytes, 0, len);
                            tex.LoadImage(imageBytes);
                            cameraMaterial.mainTexture = tex;
                            imageBytes = null;
                        } else {
                            Debug.Log("Couldnt obtain the image from the packet. Size: " + recBytes.Length + " bytes.");  
                            break;
                        }
                        //gotImage = true;

                    break;
                    default:
                        //workaround for UDP packets, which does not contain the size (image) and the type (sound)
                        if(recBytes.Length > 4200) //possibly image
                        {
                            tex.LoadImage(recBytes);
                            cameraMaterial.mainTexture = tex;
                        } else if(recBytes.Length == 4000) //likely to be a sound
                        {
                            SoundScript.adjustSoundAndPlay(recBytes, recBytes.Length);
                        } else {
                            Debug.Log("Received unknown " + recBytes.Length + " bytes.");  
                        }
                        break;

                    
                        //the next 4 bytes are the sound length
                }
            }
            catch (Exception e)
            {
                Debug.Log("Error update data from client: " + e.Message);
            }
            /*
            //if we receive 4k bytes, it is a sound
            if(recBytes.Length == 4000)
            {
                //play the sound we just received
                SoundScript.adjustSoundAndPlay(recBytes, recBytes.Length);
            } else if(recBytes.Length > 4000){
                //Update the image
                tex.LoadImage(recBytes);
                cameraMaterial.mainTexture = tex;
                //gotImage = true;
            } else {
              Debug.Log("Received unknown " + recBytes.Length + " bytes.");  
            }
            */
        }
        int bytePos = 1; //the index 0 will be set after we decide what we will be sending

        //foreach (var message in connection.getMessages()) Debug.Log(message);
        //first, lets find out if there is any sound to send
        if(SoundScript.soundToSend(udpSendBytes, ref bytePos))
        {
            //in this case, we gonna send the motor steps even if there is no update
            //bytePos = 2000;
            udpSendBytes[0] = (byte)Connection.PacketType.SOUND_SHORT;//SOUND_AND_CHANGE_12_MOTORS_STEPS;
            //updateControlersAngles();
            //note that bytePos was incremented by SoundScript.soundToSend
            //updateSendBytes(udpSendBytes, ref bytePos);
            connection.Send(udpSendBytes, bytePos);
        } else if (Time.time >= nextTime)
        {
/*
            //first, lets find out if there is any sound to send
            if(SoundScript.soundToSend(udpSendBytes, ref bytePos))
            {
                //in this case, we gonna send the motor steps even if there is no update
                //bytePos = 2000;
                udpSendBytes[0] = (byte)Connection.PacketType.SOUND_AND_CHANGE_12_MOTORS_STEPS;
                updateControlersAngles();
                //note that bytePos was incremented by SoundScript.soundToSend
                updateSendBytes(udpSendBytes, ref bytePos);
                connection.Send(udpSendBytes, bytePos);
            } else {//*/
                //in this case, we gonna send the motor steps even if there is no update
                udpSendBytes[0] = (byte)Connection.PacketType.CHANGE_12_MOTORS_STEPS;
                //update send bytes based on the Controller's position
                if(updateControlersAngles())
                {
                    updateSendBytes(udpSendBytes, ref bytePos);
                    connection.Send(udpSendBytes, bytePos);
                }
            //}

            //If it is we provide the Send method the game name and time
            //These will be bundled with the server's IP address (which is generated in StartConnection)
            //to be included in the broadcast packet
            //connection.Send(nextTime, ClientServerInfo.GameName);
            nextTime += (1.0f/timesPerSecond);
        }
    }
 
    void OnDestroy()
    {
        connection.Stop();
    }
}
