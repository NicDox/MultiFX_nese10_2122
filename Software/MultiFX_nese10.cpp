#include "daisy_seed.h"
#include "daisysp.h"
#include <stdio.h>
#include <string.h>
#include "dev/oled_ssd130x.h" 
#include "pin_definitions.h"

#define MAX_DELAY static_cast<size_t>(48000 * 2.5f) // Set max delay time to 0.75 of samplerate.


using namespace daisy;
using namespace daisysp;
using OledDisp = OledDisplay<SSD130xI2c128x64Driver>;


static DaisySeed hw;

static Phaser										phas;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS 	del;
static Tremolo										trem;
static Parameter 									deltime;

static OledDisp display;

float phas_LFO_freq = 2, phas_LFO_depth = 0.4;
float trem_LFO_freq = 2, trem_LFO_depth = 0.4,  dryWet = 0.5;
float global_volume = 0.5;
int currentEffect = 0;
bool effectOn[NUM_OF_EFFECTS] = {true, false, false}; //Turn ON Phaser initially
bool BP_On = 0;

enum FX_types {
	PHAS,		//0
	DEL,		//1	
	TREM,		//2
	// DRY,
	// ALL,
};



void GetPhaserSamples(float &in);
void GetDelaySamples(float &in);
void GetTremoloSamples(float &in);

void UpdateControls();
void UpdateKnobs();
void UpdateLeds(dsy_gpio led_pin);
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
		if (BP_On){
			out[0][i] = in[0][i];
		}
		else{
			for (int j = 0; j <= TREM; j++){
				float originalSig = sig;

				if (effectOn[j]){
					GetSamples(sig, (FX_types)j);
					out[0][i] = (sig * dryWet + originalSig * (1 - dryWet)) * global_volume;
				}
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
	AdcChannelConfig adcConfig[ADC_PINS];
	adcConfig[0].InitSingle(hw.GetPin(PIN_VOL));	//VOLUME
	adcConfig[1].InitSingle(hw.GetPin(PIN_FX1));	//Tune FX_par1
													//Tune FX_par2 ??
													//Tune dry/wet percentage ??


	hw.adc.Init(adcConfig, ADC_PINS);
	hw.adc.Start();


	//Configure GPIO pins for buttons
	Switch button_BP;
	Switch button_SW_NEXT;
	Switch button_SW_BACK;

	const int PB_SR = 1000; //Sample rate to update pushbuttons
	button_BP.Init(hw.GetPin(PIN_TOGGLE_BP), PB_SR);
	button_SW_NEXT.Init(hw.GetPin(PIN_NEXT_FX), PB_SR);
	button_SW_BACK.Init(hw.GetPin(PIN_BACK_FX), PB_SR); 

	//Configure GPIO pin for Bypass LED
	dsy_gpio led_BP;

	led_BP.pin = hw.GetPin(PIN_LED_BP);
	led_BP.mode = DSY_GPIO_MODE_OUTPUT_PP;
	led_BP.pull = DSY_GPIO_PULLUP;
	dsy_gpio_init(&led_BP);
	// led_BP.Init(PIN_LED_BP, Mode::Input, Pull::PULLUP, Speed::Slow);

	//Configure OLED display -- NON CE NE È BISOGNO (?)
	//OledDisp::Config disp_cfg;
	//disp_cfg.driver_config.transport_config.i2c_address;
	//disp_cfg.driver_config.transport_config.i2c_config


	//Set params
	phas.SetLfoFreq(phas_LFO_freq);
	phas.SetLfoDepth(phas_LFO_depth);

	trem.SetFreq(trem_LFO_freq);
	trem.SetDepth(trem_LFO_depth);


	//Control dry/wet

	//Callback start
	hw.StartAudio(AudioCallback);
	
	while(1) {
		UpdateScreen();
        //Debounce the button
        button_BP.Debounce();
        //If the button is pressed, turn the LED on
        if (button_BP.Pressed()){
			dsy_gpio_write(&led_BP, !dsy_gpio_read(&led_BP));
			BP_On = !BP_On;
		}
        button_SW_NEXT.Debounce();
        //If the button is pressed, turn the LED on
        if (button_SW_NEXT.Pressed()){
			int previousEffect = currentEffect;
			currentEffect = abs(currentEffect + 1) % NUM_OF_EFFECTS;
			effectOn[previousEffect] = false;
			effectOn[currentEffect] = true;
		}

		button_SW_BACK.Debounce();
		if (button_SW_BACK.Pressed()){
			int previousEffect = currentEffect;
			if (previousEffect == 0){
				currentEffect = NUM_OF_EFFECTS-1;
			}
			else {
				currentEffect--;
			}
			effectOn[previousEffect] = false;
			effectOn[currentEffect] = true;			
		}

		System::Delay(6);

		//UpdateLeds();
	}
}


//UpdateKnobs
void UpdateKnobs(){
	trem.SetFreq(hw.adc.GetFloat(1));
	phas.SetLfoFreq(hw.adc.GetFloat(1));
	global_volume = hw.adc.GetFloat(0);
}


//UpdateSwtiches

//UpdateLeds
void UpdateLeds(dsy_gpio *led_pin){
	dsy_gpio_toggle(led_pin);
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
