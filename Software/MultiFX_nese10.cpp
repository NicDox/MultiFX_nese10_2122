#include "daisy_seed.h"
#include "daisysp.h"
#include <stdio.h>
#include <string.h>
#include "dev/oled_ssd130x.h" 

// Set max delay time to 0.75 of samplerate.
#define MAX_DELAY static_cast<size_t>(48000 * 2.5f)

using namespace daisy;
using namespace daisysp;

using MyOledDisplay = OledDisplay<SSD130xI2c128x64Driver>;

static DaisySeed hw;

static Phaser										phas;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS 	del;
static Tremolo										trem;
static Parameter deltime;

static MyOledDisplay display;

float currentDelay, feedback, delayTarget, cutoff, dryWet[3];
bool effectOn[3];

enum FX_types {
	PHAS,
	DEL,
	TREM,
	// DRY,
	// ALL,
};

void GetPhaserSamples(float &in);
void GetDelaySamples(float &in);
void GetTremoloSamples(float &in);

void UpdateControls();
void UpdateLeds();
void UpdateScreen();
void GetSamples(float &in, FX_types type){
	switch(type)
	{
		case PHAS: GetPhaserSamples(in); break;
		case DEL: GetDelaySamples(in); break;
		case TREM: GetTremoloSamples(in); break;
		default: break;
	}
}



FX_types which_FX;	//Here is stored the value of the actual effect

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	UpdateControls(); 	//Each time a new audio buffer is to be processed, 
						//we check state of controls.

	for (size_t i = 0; i < size; i++) //Cycle each sample contained inside buffer
	{
		float sig = in[0][i];
		
		for (int j = 0; j <= TREM; j++){
			float originalSig = sig;

			if (effectOn[j]){
				GetSamples(sig, (FX_types)j);
				out[0][i] = sig * dryWet[i] + originalSig * (1 - dryWet[i]);
			}
		} 
	}
}

int main(void)
{
	//Init hw and FX
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	float sr = hw.AudioSampleRate();
	phas.Init(sr);
	del.Init();
	trem.Init(sr);

	AdcChannelConfig adcConfig;
	adcConfig.InitSingle(hw.GetPin(21));

	hw.adc.Init(&adcConfig, 1);

	effectOn[0] = effectOn[1] = effectOn[2] = false;

	//Set params


	//Control dry/wet

	//Callback start
	
	//hw.adc.Init(&adcConfig, 1);

	hw.StartAudio(AudioCallback);
	
	while(1) {
		UpdateScreen();
		UpdateLeds();
	}
}


//UpdateKnobs

//UpdateSwtiches

//UpdateLeds
void UpdateLeds(){

}

//UpdateScreen
void UpdateScreen(){

}

//UpdateControls
void UpdateControls(){

}

//GetPhaserSample
void GetPhaserSamples(float &in){
	in = phas.Process(in);
}
//GetDelaySample
void GetDelaySamples(float &in){
	in = del.Read();
}
//GetTremoloSample
void GetTremoloSamples(float &in){
	in = trem.Process(in);
}
