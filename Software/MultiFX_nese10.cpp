#include "daisy_seed.h"
#include "daisysp.h"
#include <stdio.h>
#include <string.h>
#include "dev/oled_ssd130x.h" 
#include "pin_definitions.h"
//#include "DaisyLibrary.h"

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
bool effectOn[NUM_OF_EFFECTS] = {true, false, false}; //Turn ON Delay initially
bool BP_On = 0;

enum FX_types {//CAMBIA ORDINE
	DEL,		//0
	TREM,		//1	
	PHAS,		//2
	// DRY,
	// ALL,
};




void init_MultiFx();
void init_display();
void read_volume();
void read_percentageFx();
void UpdateLeds(dsy_gpio led_pin);
void updateDisplay();

void GetPhaserSamples(float &in);
void GetDelaySamples(float &in);
void GetTremoloSamples(float &in);
void GetSamples(float &in, FX_types type){
	switch(type)
	{
		case DEL: GetDelaySamples(in); break;
		case TREM: GetTremoloSamples(in); break;
		case PHAS: GetPhaserSamples(in); break;
		default: break;
	}
}

void bypass();
void nextFx();
void backFx();

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	read_volume(); 	//Each time a new audio buffer is to be processed, 
						//we check state of controls.
	read_percentageFx();

	for (size_t i = 0; i < size; i++) //Cycle each sample contained inside buffer
	{
		float sig = in[0][i];
		if (BP_On){
			out[0][i] = in[0][i];
		}
		else{
			for (int j = 0; j <= PHAS; j++){
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
	init_MultiFx();
	init_display();

	float sr = hw.AudioSampleRate();
	phas.Init(sr);
	del.Init();
	trem.Init(sr);

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
	dsy_gpio_write(&led_BP, 0); //Initially BP is off so also the led is off
	

	//Set params
	phas.SetLfoFreq(phas_LFO_freq);
	phas.SetLfoDepth(phas_LFO_depth);

	trem.SetFreq(trem_LFO_freq);
	trem.SetDepth(trem_LFO_depth);


	//Callback start
	hw.StartAudio(AudioCallback);
	
	while(1) {
		updateDisplay();
        //Debounce the button
        button_BP.Debounce();
        //If the button is pressed, turn the LED on
        if (button_BP.Pressed()){
			bypass();
			UpdateLeds(led_BP);
		}
        
		button_SW_NEXT.Debounce();
        //If the button is pressed, turn the LED on
        if (button_SW_NEXT.Pressed()){
			nextFx();
		}

		button_SW_BACK.Debounce();
		if (button_SW_BACK.Pressed()){
			backFx();
		}

		System::Delay(5);

	}
}

void init_MultiFx(){
	hw.Init(); 
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

	//Configure ADC pins for pots
	AdcChannelConfig adcConfig[ADC_PINS];
	adcConfig[0].InitSingle(hw.GetPin(PIN_VOL));	//VOLUME
	adcConfig[1].InitSingle(hw.GetPin(PIN_FX1));	//Tune FX_par1

	hw.adc.Init(adcConfig, ADC_PINS);
	hw.adc.Start();
}

void init_display(){
    OledDisp::Config disp_cfg;
    disp_cfg.driver_config.transport_config.i2c_address               = 0x78;
    disp_cfg.driver_config.transport_config.i2c_config.periph         = I2CHandle::Config::Peripheral::I2C_1;
    disp_cfg.driver_config.transport_config.i2c_config.speed          = I2CHandle::Config::Speed::I2C_100KHZ;
    disp_cfg.driver_config.transport_config.i2c_config.mode           = I2CHandle::Config::Mode::I2C_MASTER;
    disp_cfg.driver_config.transport_config.i2c_config.pin_config.scl = {DSY_GPIOB, 8};//hw.GetPin(12);
    disp_cfg.driver_config.transport_config.i2c_config.pin_config.sda = {DSY_GPIOB, 9};//hw.GetPin(13);
    /** And Initialize */
    display.Init(disp_cfg);
}

void read_volume(){

	global_volume = hw.adc.GetFloat(0);

}

void read_percentageFx(){
	trem_LFO_freq = hw.adc.GetFloat(1);
	phas_LFO_freq = trem_LFO_freq;
	trem.SetFreq(trem_LFO_freq);
	phas.SetLfoFreq(phas_LFO_freq);
}


//UpdateLeds
void UpdateLeds(dsy_gpio *led_pin){
	dsy_gpio_toggle(led_pin);
}


//UpdateScreen
void updateDisplay(){
	char strbuff[128];
	switch(currentEffect){
		case 0: sprintf(strbuff, "Effect: Delay\nDel_time: %f ms\nVolume: %f\n", deltime, global_volume);
		case 1: sprintf(strbuff, "Effect: Tremolo\nLFO_freq: %f Hz\nVolume: %f\n",trem_LFO_freq, global_volume);
		case 2: sprintf(strbuff, "Effect: Phaser\nLFO_freq: %f Hz\nVolume: %f\n",phas_LFO_freq, global_volume);
	}
	display.Fill(true);
	display.SetCursor(0, 0);
	display.WriteString(strbuff, Font_11x18, false);
	display.Update();
}

void bypass(){
		BP_On = !BP_On;
}

void nextFx(){
		int previousEffect = currentEffect;
		currentEffect = abs(currentEffect + 1) % NUM_OF_EFFECTS;
		effectOn[previousEffect] = false;
		effectOn[currentEffect] = true;
}

void backFx(){
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


