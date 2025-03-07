using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

public class SoundScript : MonoBehaviour
{
    static SoundScript soundScript;
    double nextStartTime; //used to determine
    public AudioSource []sound;
    //AudioClip []clip;
    int currentSound;
    const int frequency = 8000;
    //how many samples we must sample after a trigger is identified
    const int SAMPLESAFTERTRIGGER = 12000; 
    const int SAMPLESCOUNTSPEAKER = 4000;//4000;//16000;
    public static int SAMPLESCOUNTMIC = 4000;//1334;//limited by Ethernet/Wifi MTU. THIS VALUE WILL BE CHANGE BY CONNECTION LOGIC TO 1334 if UDP IS SELECTED.
    const int SOUNDQUEUESIZE = 4;
    const int SOUNDGAIN = 1; //gain of the microphone
    static int samplesSendingAfterTrigger = 0;
    static AudioClip microphoneInput;
    //bool microphoneInitialized;
    //public float sensitivity;
    //public bool flapped;
    //List<float> micSamples;

    
    //int lastMicPostion = 0;
    static double nextTimeMicRead;
    static float[] microphoneData;

    //int channels = 1;
    //public AudioSource sampleSound;
    // Start is called before the first frame update
    void Start()
    {
        soundScript = this;
        nextStartTime = 0;
        currentSound = 0;
        sound = new AudioSource[SOUNDQUEUESIZE];
        //create the AudioSources
        for(int i = 0; i < SOUNDQUEUESIZE; i++)
        {
            sound[i] = gameObject.AddComponent<AudioSource>();
            sound[i].clip = AudioClip.Create("TempClip"+i, SAMPLESCOUNTSPEAKER, 1, frequency, false, false);
        }
        /*
        foreach (var device in Microphone.devices)
        {
            Debug.Log("Name: " + device);
        }*/

        //micSamples = new List<float>();
        microphoneData = new float[SAMPLESCOUNTMIC];

        //init microphone input
        if (Microphone.devices.Length>0){
            microphoneInput = Microphone.Start(Microphone.devices[0],true,100,8000);
            //microphoneInitialized = true;
        }
        
        /*
        adjustSound(tmpSound, tmpSound.Length);
        playData(tmpSound);
        playData(tmpSound);
        playData(tmpSound);
        playData(tmpSound);//*/
    }
    
    public static void adjustSoundAndPlay(byte []byteSamples, int length)
    {///*
        
        float []samples = new float[length];
        float min = 10000, max = 0, dif;
        int i;
        for(i = 0; i < length; i++)
        {
            samples[i] = byteSamples[i];
            if(samples[i] < min)
                min = samples[i];
            if(samples[i] > max)
                max = samples[i];
        }

        dif = max - min;
        for(i = 0; i < length; i++)
        {
            samples[i] = samples[i] - min;
            samples[i] = (samples[i]*((float)2/dif))-1;//from -1 to 1
        }
        soundScript.playData(samples);
    }
    //void playData(List<int> samples, int frequency)
    void playData(float []samples)
    {

		sound[currentSound].clip.SetData(samples, 0); //samples.ToArray()
        double time = AudioSettings.dspTime;
        //is the next start time in the past?
        if(nextStartTime < AudioSettings.dspTime)
            nextStartTime = AudioSettings.dspTime;
        //sound[currentSound].Stop();
        sound[currentSound].PlayScheduled(nextStartTime);

        //update the next start time
        nextStartTime +=  ((double)SAMPLESCOUNTSPEAKER/(double)frequency);

        currentSound++;
        if(currentSound == SOUNDQUEUESIZE)
            currentSound = 0;
        //sound.audio.Play();
    }
    public static bool soundToSend(byte []data, ref int bytePos)
    {        
        
        //if it is not yet time to sample
        if(nextTimeMicRead > AudioSettings.dspTime)
            return false;

        int bkpPos = bytePos;

        nextTimeMicRead = AudioSettings.dspTime + ((double)SAMPLESCOUNTMIC/(double)frequency);
        
        int micPosition = Microphone.GetPosition(Microphone.devices[0]);

        if(micPosition < SAMPLESCOUNTMIC+1)
            return false;
            
        //get the last dec samples
        microphoneInput.GetData(microphoneData, micPosition-(SAMPLESCOUNTMIC+1));
        //microphoneInput.RemoveRange(0,dec);
        //int size = microphoneInput.samples;
        
        float levelMax = 0;
        for (int i = 0; i < SAMPLESCOUNTMIC; i++) {
            //waveData[i] = (waveData[i]+1)*255; // from 0 to 255
            float wavePeak = microphoneData[i] * microphoneData[i];
            if (levelMax < wavePeak) {
                levelMax = wavePeak;
            }
            microphoneData[i] *= SOUNDGAIN;
            if(microphoneData[i] < -1)
                microphoneData[i] = -1;
            else if(microphoneData[i] > 1)
                microphoneData[i] = 1;

            data[bytePos] = (byte)(((microphoneData[i]+1)*255)/2); // from 0 to 255
            //data[bytePos] = (byte)(((int)Mathf.Round(data[bytePos]/16.0f))*16);
            bytePos++;
        }

        ///*
        
        byte min = 115, max = 139, dif;
        int val;
        //min = 115;
        //max = 139;
        for(int i = 0; i < SAMPLESCOUNTMIC; i++)
        {
            //small noise filter
            //if(data[bkpPos+i] >= 126 && data[bkpPos+i] <= 128)
            //    data[bkpPos+i] = 127;

            if(data[bkpPos+i] < min)
                min = data[bkpPos+i];
            if(data[bkpPos+i] > max)
                max = data[bkpPos+i];
        }//*/
        ///*
        dif = (byte)(max - min);
        ///*
        for(int i = 0; i < SAMPLESCOUNTMIC; i++)
        {
            val = (data[bkpPos+i] - min);
            if(val < 0)
                val = 0;
            val = (byte)(val*((double)(255)/dif));
            if(val > 255)
                val = 255;
            data[bkpPos+i] = (byte)val;
            //printf("Sample[%d]: %d mV\n", i, samples[i]);
        }//*/

        float level = Mathf.Sqrt(Mathf.Sqrt(levelMax));
        
        //should we update the value of the samples we want to send?
        if(level > 0.25)
        //if(level > 0.55)
        //if(level > 1)
            samplesSendingAfterTrigger = SAMPLESAFTERTRIGGER;

        //do we want to keep sending samples?
        if(samplesSendingAfterTrigger <= 0)
        {
            //restore the old bytePos since we will not consider the audio
            bytePos = bkpPos;
            return false;
        } else {
            //decrement the samples 
            samplesSendingAfterTrigger -= SAMPLESCOUNTMIC;
            return true;
        }
    }
}