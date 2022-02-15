#include "daisy_seed.h"
#include "daisysp.h"
#include <stdio.h>
#include <string.h>
#include "dev/oled_ssd130x.h" 

// Set max delay time to 0.75 of samplerate.
#define MAX_DELAY static_cast<size_t>(48000 * 2.5f)
#define PIN_VOL			3		
#define PIN_FX1			4
#define PIN_NEXT_FX		14
#define PIN_BACK_FX		13
#define PIN_TOGGLE_BP	15
#define PIN_LED_BP 		17


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

	//Configure ADC pins for pots
	AdcChannelConfig adcConfig;
	adcConfig.InitSingle(hw.GetPin(PIN_VOL));	//VOLUME
	adcConfig.InitSingle(hw.GetPin(PIN_FX1));	//Tune FX_par1
										//Tune FX_par2 ??
										//Tune dry/wet percentage ??

	hw.adc.Init(&adcConfig, 2);

	//Configure GPIO pins for buttons
	Switch button_BP;
	Switch button_SW_NEXT;
	Switch button_SW_BACK;

	const int PB_SR = 1000; //Sample rate to update pushbuttons
	button_BP.Init(hw.GetPin(PIN_TOGGLE_BP), PB_SR);
	button_SW_NEXT.Init(hw.GetPin(PIN_NEXT_FX), PB_SR);
	button_SW_BACK.Init(hw.GetPin(PIN_BACK_FX), PB_SR); 

	dsy_gpio led_BP;

	led_BP.pin = hw.GetPin(PIN_LED_BP);
	led_BP.mode = DSY_GPIO_MODE_OUTPUT_PP;
	led_BP.pull = DSY_GPIO_PULLUP;
	dsy_gpio_init(&led_BP);
	// led_BP.Init(17, Mode::Input, Pull::PULLUP, Speed::Slow);

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
